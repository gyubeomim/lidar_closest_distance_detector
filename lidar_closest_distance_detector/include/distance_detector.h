/*
 * Copyright Dynamic Robotic Systems (DYROS) Laboratory, Seoul National University. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *
 *        * Redistributions in binary form must reproduce the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided
 *          with the distribution.
 *
 *        * Neither the name of the DYROS Dynamic Robotic Systems Lab, Seoul National University.
 *          nor the names of its contributors may be used to endorse or promote products
 *          derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
