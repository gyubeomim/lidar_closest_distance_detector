#include "filtering.h"

namespace dyrosvehicle {

// /points_obstacle callback function
void FilteringNodelet::points_obstacle_callback(const VPointCloud::ConstPtr &msg,
                                                ros::NodeHandle priv_nh) {
  if(msg->points.empty()) return;

  double radius = priv_nh.param<double>("obstacle_outrem_radius", 3.0);
  int minNeighbors = priv_nh.param<int>("obstacle_outrem_minneighbors", 10);

  pcl::RadiusOutlierRemoval<VPoint> outrem_obs;
  outrem_obs.setInputCloud(msg);
  outrem_obs.setRadiusSearch(radius);
  outrem_obs.setMinNeighborsInRadius (minNeighbors);
  outrem_obs.filter (*msg_filtered);

  msg_filtered->header.frame_id = "camera_init";
  // publish to /points_obstacle_filtered
  pub_obstacle_filtered.publish(msg_filtered);
}

// /current_convex_polygon callback function
void FilteringNodelet::polygon_array_callback(const jsk_recognition_msgs::PolygonArray::ConstPtr &msg) {
  if(!arr_polygon.polygons.empty())
    arr_polygon.polygons.clear();

  for(int i=0; i<msg->polygons.size(); i++) {
    arr_polygon.polygons.push_back(msg->polygons[i]);
  }
}

void FilteringNodelet::onInit()
{
  nh = getNodeHandle();
  priv_nh = getPrivateNodeHandle();

  msg_filtered = VPointCloud::Ptr(new VPointCloud);

  sub_obstacle = nh.subscribe<VPointCloud>("points_obstacle", 1, boost::bind(&FilteringNodelet::points_obstacle_callback,this,_1,priv_nh));
  sub_polygon_arr = nh.subscribe<jsk_recognition_msgs::PolygonArray>("current_convex_polygon", 1, &FilteringNodelet::polygon_array_callback, this);

  pub_obstacle_filtered = nh.advertise<VPointCloud>("points_obstacle_filtered", 1);
}

} // namespace dyrosvehicle
