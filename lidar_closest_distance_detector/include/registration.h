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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <algorithm>

namespace dyrosvehicle {

using namespace std;

class RegistrationNodelet : public nodelet::Nodelet {
 private:
  bool loam_toggle;

  //convenient typedefs
  typedef pcl::PointXYZI VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  VPointCloud::Ptr msg_tf;
  VPointCloud::Ptr msg_ptfiltered;
  VPointCloud::Ptr msg_vg;
  VPointCloud::Ptr msg_sum;
  VPointCloud::Ptr lpc_ptr2;

  string velodyne_frame;

  tf::StampedTransform tf_globalvariable;

  ros::Publisher processed_pub;
  ros::Publisher processed_pub2;
  ros::Publisher pub_obstacle;
  ros::Publisher pub_obstacle_registered;
  ros::Subscriber sub_spacefilter;
  ros::Subscriber sub_obstacle_filtered;
  ros::Subscriber sub_loam_toggle;

  tf::TransformListener listener;
  tf::StampedTransform transform_;

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

 public:
  virtual void onInit();

  RegistrationNodelet(){}

  ~RegistrationNodelet() {}

  void addObstaclePointCloud(VPointCloud& lpc, const VPointCloud& rpc);
  void points_obstacle_filtered_callback(const VPointCloud::ConstPtr &msg,
                                         tf::TransformListener *listener,
                                         tf::StampedTransform *transform_);

  void spacefilter_callback(const VPointCloud::ConstPtr &msg,
                            ros::NodeHandle priv_nh);
  void loam_toggle_callback(const std_msgs::Bool::ConstPtr &msg);
};


} // namespace dyrosvehicle

PLUGINLIB_EXPORT_CLASS(dyrosvehicle::RegistrationNodelet, nodelet::Nodelet)
