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
