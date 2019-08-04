#include "distance_detector.h"

namespace dyrosvehicle {

void DistanceDetectorNodelet::polygon_array_callback(const jsk_recognition_msgs::PolygonArray::ConstPtr &msg,
                            tf::TransformListener *listener,
                            tf::StampedTransform *transform_) {
  if(msg->polygons.empty()) return;

  try {
    listener->lookupTransform("camera_init", "camera", ros::Time(0), *transform_);
  }
  catch (tf::TransformException ex) {
    ROS_WARN("%s", ex.what());
  }

  geometry_msgs::Point32 pt;
  geometry_msgs::PolygonStamped ego_vehicle;
  if(ego_vehicle.polygon.points.empty()) {
    tf::Vector3 pt1 = *transform_ * tf::Vector3(3.0,   1.0, 0);
    tf::Vector3 pt2 = *transform_ * tf::Vector3(3.0,   0, 0);
    tf::Vector3 pt3 = *transform_ * tf::Vector3(3.0,  -1.0, 0);
    tf::Vector3 pt4 = *transform_ * tf::Vector3(0,  -1.0, 0);
    tf::Vector3 pt5 = *transform_ * tf::Vector3(-2.25,  -1.0, 0);
    tf::Vector3 pt6 = *transform_ * tf::Vector3(-2.25,  0, 0);
    tf::Vector3 pt7 = *transform_ * tf::Vector3(-2.25, 1.0, 0);
    tf::Vector3 pt8 = *transform_ * tf::Vector3(0, 1.0, 0);

    pt.x = pt1.getX(); pt.y = pt1.getY();
    ego_vehicle.polygon.points.push_back(pt);

    pt.x = pt2.getX(); pt.y = pt2.getY();
    ego_vehicle.polygon.points.push_back(pt);

    pt.x = pt3.getX(); pt.y = pt3.getY();
    ego_vehicle.polygon.points.push_back(pt);

    pt.x = pt4.getX(); pt.y = pt4.getY();
    ego_vehicle.polygon.points.push_back(pt);

    pt.x = pt5.getX(); pt.y = pt5.getY();
    ego_vehicle.polygon.points.push_back(pt);

    pt.x = pt6.getX(); pt.y = pt6.getY();
    ego_vehicle.polygon.points.push_back(pt);

    pt.x = pt7.getX(); pt.y = pt7.getY();
    ego_vehicle.polygon.points.push_back(pt);

    pt.x = pt8.getX(); pt.y = pt8.getY();
    ego_vehicle.polygon.points.push_back(pt);

    ego_vehicle.header.frame_id = "camera_init";
    pub_ego_poly.publish(ego_vehicle);
  }

  double min_dist, dist;
  min_dist = 9999;
  for(int i=0; i<msg->polygons.size(); i++) {
    for(int j=0; j<msg->polygons[i].polygon.points.size(); j++) {
      double x = msg->polygons[i].polygon.points[j].x;
      double y = msg->polygons[i].polygon.points[j].y;
      for(int k=0; k<ego_vehicle.polygon.points.size(); k++) {
        double ego_x = ego_vehicle.polygon.points[k].x;
        double ego_y = ego_vehicle.polygon.points[k].y;

        dist = std::sqrt(std::pow(x-ego_x, 2) + std::pow(y-ego_y, 2));

        if(dist > 20) break;

        if(dist < min_dist) {
          min_dist = dist;

          marker_msg.points.clear();
          geometry_msgs::Point p_start;
          p_start.x = x;
          p_start.y = y;
          p_start.z = 18;
          marker_msg.points.push_back(p_start);
          geometry_msgs::Point p_end;
          p_end.x = ego_x;
          p_end.y = ego_y;
          p_end.z = 18;
          marker_msg.points.push_back(p_end);

          marker_msg.header.frame_id = "camera_init";
          marker_msg.ns = "line";
          marker_msg.id = 0;
          marker_msg.type = visualization_msgs::Marker::LINE_LIST;
          marker_msg.action = visualization_msgs::Marker::ADD;
          marker_msg.scale.x = 0.1; marker_msg.scale.y = 0.1; marker_msg.scale.z = 0.1;                           // scale
          marker_msg.color.a = 1.0; marker_msg.color.r = 1.0; marker_msg.color.g = 1.0; marker_msg.color.b = 1.0; // color
        }
      }
      if(dist > 20) continue;
    }
  }
  arr_marker_msg.markers.push_back(marker_msg);
  pub_closest.publish(arr_marker_msg);
  arr_marker_msg.markers.clear();

  ego_vehicle.polygon.points.clear();
}

void DistanceDetectorNodelet::onInit()
{
  nh = getNodeHandle();
  priv_nh = getPrivateNodeHandle();

  sub_polygon_arr = nh.subscribe<jsk_recognition_msgs::PolygonArray>("all_convex_polygon", 1, boost::bind(&DistanceDetectorNodelet::polygon_array_callback,this, _1, &listener, &transform_));

  pub_ego_poly = nh.advertise<geometry_msgs::PolygonStamped>("ego_vehicle", 1);
  pub_closest = nh.advertise<visualization_msgs::MarkerArray>("closest_distance", 1);
}

} // namespace dyrosvehicle
