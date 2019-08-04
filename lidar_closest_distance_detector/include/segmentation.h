#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include <iostream>
#include <mutex>

namespace dyrosvehicle {

using namespace std;

#ifndef TYPEDEF_POINTCLOUD
#define TYPEDEF_POINTCLOUD
//convenient typedefs
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
#endif

class SegmentationNodelet : public nodelet::Nodelet{
 private:
  VPointCloud::Ptr msg_tf;
  VPointCloud::Ptr msg_sum;
  VPointCloud::Ptr msg_sum_vg;
  VPointCloud::Ptr msg_sum_tf;
  VPointCloud::Ptr msg_sum_tf_removenan;

  ros::Publisher pub_cvx_current;
  ros::Subscriber sub_obs_filtered;

  tf::TransformListener listener;
  tf::StampedTransform transform_;

  ros::Timer timer;

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

  std::mutex mutex_seg;
 public:
  virtual void onInit();

  SegmentationNodelet() {}

  ~SegmentationNodelet() {}

  void addObstaclePointCloud(VPointCloud& lpc, const VPointCloud& rpc, ros::NodeHandle priv_nh);

  void points_obstacle_filtered_callback(const VPointCloud::ConstPtr &msg,
                                         tf::TransformListener *listener,
                                         tf::StampedTransform *transform_,
                                         ros::NodeHandle priv_nh);

  void timer_callback(const ros::TimerEvent &event);
};
} // namespace dyrosvehicle

PLUGINLIB_EXPORT_CLASS(dyrosvehicle::SegmentationNodelet, nodelet::Nodelet)
