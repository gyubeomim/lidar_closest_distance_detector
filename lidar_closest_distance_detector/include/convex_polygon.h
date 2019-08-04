#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <mutex>

namespace dyrosvehicle {
using namespace std;

class ConvexPolygonNodelet {
 private:
  //convenient typedefs
  typedef pcl::PointXYZI VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;
  typedef pcl::PointXYZRGB PointRGB;
  typedef pcl::PointCloud<PointRGB> PointCloudRGB;

  VPointCloud::Ptr lpc_ptr2;
  VPointCloud::Ptr msg_sum;
  VPointCloud::Ptr msg_tf;

  geometry_msgs::Point32 point32_all;
  geometry_msgs::PolygonStamped polygon_msg_all;
  jsk_recognition_msgs::PolygonArray arr_polygon_msg_all;
  VPointCloud::Ptr msg_z;
  vector<int> index;

  ros::Publisher pub_cvx_all;
  ros::Subscriber sub_obs_registered;

  std::mutex mutex_cvxpoly;
 public:
  ConvexPolygonNodelet(ros::NodeHandle nh, ros::NodeHandle priv_nh);

  ~ConvexPolygonNodelet(){}

  void addObstaclePointCloud(VPointCloud& lpc, const VPointCloud& rpc, ros::NodeHandle priv_nh);
  void publishAllConvexPolygon(const VPointCloud::ConstPtr msg, ros::NodeHandle priv_nh);
  void points_registered_callback(const VPointCloud::ConstPtr &msg,
                                  ros::NodeHandle priv_nh);
};
}  // end of namespace dyrosvehicle
