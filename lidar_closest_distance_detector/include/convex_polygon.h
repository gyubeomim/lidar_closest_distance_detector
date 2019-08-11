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
