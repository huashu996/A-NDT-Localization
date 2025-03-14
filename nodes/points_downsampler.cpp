#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_map>
#include <cmath>
#include <iostream>
// #include "points_downsampler.h"
#include "std_msgs/Float64.h"
#define MAX_MEASUREMENT_RANGE 120.0

ros::Publisher filtered_points_pub;
ros::Publisher double_pub;
// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static bool _output_log = false;
static std::ofstream ofs;
static std::string filename;

static std::string POINTS_TOPIC;

static pcl::PointCloud<pcl::PointXYZ> removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range)
{
  pcl::PointCloud<pcl::PointXYZ> narrowed_scan;
  narrowed_scan.header = scan.header;

  if( min_range>=max_range ) {
    ROS_ERROR_ONCE("min_range>=max_range @(%lf, %lf)", min_range, max_range );
    return scan;
  }

  double square_min_range = min_range * min_range;
  double square_max_range = max_range * max_range;

  for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    const pcl::PointXYZ &p = *iter;
    double square_distance = p.x * p.x + p.y * p.y;

    if(square_min_range <= square_distance && square_distance <= square_max_range){
      narrowed_scan.points.push_back(p);
    }
  }

  return narrowed_scan;
}
// 定义体素索引结构
struct VoxelKey {
    int x;
    int y;
    int z;

    // 重载相等比较操作符
    bool operator==(const VoxelKey &other) const {
        return (x == other.x) && (y == other.y) && (z == other.z);
    }
};

// 为 VoxelKey 定义哈希函数，使其可以用于 std::unordered_map
namespace std {
    template <>
    struct hash<VoxelKey> {
        std::size_t operator()(const VoxelKey &key) const {
            std::size_t h1 = std::hash<int>()(key.x);
            std::size_t h2 = std::hash<int>()(key.y);
            std::size_t h3 = std::hash<int>()(key.z);
            return ((h1 ^ (h2 << 1)) >> 1) ^ (h3 << 1);
        }
    };
}

// 计算点云体积的函数，只有当体素中的点数大于5时，才认为该体素占据
double computePointCloudVolume(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double voxel_size) {
    // 使用 unordered_map 来记录每个体素的点数
    std::unordered_map<VoxelKey, int> voxelCount;

    // 遍历点云中所有点，计算其所属的体素索引并累计计数
    for (const auto &pt : cloud->points) {
        int ix = static_cast<int>(std::floor(pt.x / voxel_size));
        int iy = static_cast<int>(std::floor(pt.y / voxel_size));
        int iz = static_cast<int>(std::floor(pt.z / voxel_size));
        VoxelKey key = {ix, iy, iz};
        voxelCount[key]++;
    }

    // 统计占据的体素数量：只有当体素中的点数大于5时，才认为该体素被占据
    int occupied_voxel_count = 0;
    for (const auto &kv : voxelCount) {
        if (kv.second > 4) {
            occupied_voxel_count++;
        }
    }

    // 体积 = 占据体素数量 * (voxel_size³)
    double volume = static_cast<double>(occupied_voxel_count) * voxel_size * voxel_size * voxel_size;
    return volume;
}
static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(*input, scan);
  scan = removePointsByRange(scan, 0, MAX_MEASUREMENT_RANGE);
  //1.V
  double voxel_size = 3.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
  double volume = computePointCloudVolume(scan_ptr, voxel_size);
  voxel_leaf_size = round(pow(volume/8000.0, 1.0/3.0) * 10.0) / 10.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  sensor_msgs::PointCloud2 filtered_msg;
  if (voxel_leaf_size >= 0.1)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);
    pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
  }
  else
  {
    pcl::toROSMsg(*scan_ptr, filtered_msg);
  }
 
  filtered_msg.header = input->header;
  filtered_points_pub.publish(filtered_msg);
  std_msgs::Float64 msg;
  msg.data = voxel_leaf_size;
	std::cout << "------------------------------------------------" << std::endl;
	std::cout << "点云体积: " << volume << std::endl;
	std::cout <<"NUM：  "<<filtered_scan_ptr->points.size()<<std::endl;
  ROS_INFO("Down-voxel-size: %f", msg.data);
  double_pub.publish(msg);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voxel_grid_filter");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("points_topic", POINTS_TOPIC);
  private_nh.getParam("output_log", _output_log);

  private_nh.param<double>("leaf_size", voxel_leaf_size, 2.0);
  ROS_INFO_STREAM("Voxel leaf size is: "<<voxel_leaf_size);
  if(_output_log == true){
	  char buffer[80];
	  std::time_t now = std::time(NULL);
	  std::tm *pnow = std::localtime(&now);
	  std::strftime(buffer,80,"%Y%m%d_%H%M%S",pnow);
	  filename = "voxel_grid_filter_" + std::string(buffer) + ".csv";
	  ofs.open(filename.c_str(), std::ios::app);
  }

  // Publishers
  filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1000);
  double_pub = nh.advertise<std_msgs::Float64>("/voxel_leaf_size", 1000);

  // Subscribers
  ros::Subscriber scan_sub = nh.subscribe(POINTS_TOPIC, 1000, scan_callback);

  ros::spin();

  return 0;
}
