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
