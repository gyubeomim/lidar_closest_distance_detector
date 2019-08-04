#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <iostream>
#include <mutex>
#include <thread>

namespace dyrosvehicle {

using namespace std;

//convenient typedefs
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

typedef Eigen::Vector2d vec2d;
typedef Eigen::Vector3d vec3d;
typedef Eigen::Vector3f vec3f;
typedef Eigen::Vector4f vec4f;

class FilteringNodelet : public nodelet::Nodelet {
 private:
  bool TEST_WITHOUT_CONVEX_FILTERING;

  VPointCloud::Ptr msg_filtered;

  ros::Publisher pub_obstacle_filtered;
  ros::Subscriber sub_tf;
  ros::Subscriber sub_obstacle;
  ros::Subscriber sub_polygon_arr;

  jsk_recognition_msgs::PolygonArray arr_polygon;

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

  std::mutex mutex_filter;
 public:
  virtual void onInit();

  FilteringNodelet(){}

  ~FilteringNodelet() {};

  void points_obstacle_callback(const VPointCloud::ConstPtr &msg,
                                ros::NodeHandle priv_nh);

  void polygon_array_callback(const jsk_recognition_msgs::PolygonArray::ConstPtr &msg);

};
} // namespace dyrosvehicle

PLUGINLIB_EXPORT_CLASS(dyrosvehicle::FilteringNodelet, nodelet::Nodelet)
