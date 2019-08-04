#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <string>
#include <Eigen/Dense>

namespace dyrosvehicle {
using namespace std;

class DistanceDetectorNodelet : public nodelet::Nodelet {
 private:
  typedef Eigen::Vector2d vec2d;
  typedef Eigen::Vector3d vec3d;
  typedef Eigen::Vector4d vec4d;
  typedef Eigen::Vector3f vec3f;
  typedef Eigen::Vector4f vec4f;
  typedef Eigen::VectorXd vecXd;
  typedef Eigen::Matrix3f mat3f;
  typedef Eigen::MatrixXd matXd;

  visualization_msgs::Marker marker_msg;
  visualization_msgs::MarkerArray arr_marker_msg;

  ros::Publisher pub_closest;
  ros::Publisher pub_ego_poly;
  ros::Subscriber sub_polygon_arr;

  tf::TransformListener listener;
  tf::StampedTransform transform_;

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
 public:
  virtual void onInit();

  DistanceDetectorNodelet() {}

  ~DistanceDetectorNodelet(){}

  void polygon_array_callback(const jsk_recognition_msgs::PolygonArray::ConstPtr &msg,
                              tf::TransformListener *listener,
                              tf::StampedTransform *transform_);
};


} // namespace dyrosvehicle

PLUGINLIB_EXPORT_CLASS(dyrosvehicle::DistanceDetectorNodelet, nodelet::Nodelet)
