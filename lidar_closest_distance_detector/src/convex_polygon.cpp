#include "convex_polygon.h"

namespace dyrosvehicle {

void ConvexPolygonNodelet::addObstaclePointCloud(VPointCloud& lpc, const VPointCloud& rpc, ros::NodeHandle priv_nh) {
  // Make the resultant point cloud take the newest stamp
  if (rpc.header.stamp > lpc.header.stamp)
    lpc.header.stamp = rpc.header.stamp;

  size_t nr_points = lpc.points.size ();
  lpc.points.resize (nr_points + rpc.points.size ());

  for (size_t i = nr_points; i < lpc.points.size (); ++i) {
      lpc.points[i] = rpc.points[i - nr_points];
  }

  lpc.width    = static_cast<uint32_t>(lpc.points.size ());
  lpc.height   = 1;
  if (rpc.is_dense && lpc.is_dense)
    lpc.is_dense = true;
  else
    lpc.is_dense = false;
}

void ConvexPolygonNodelet::publishAllConvexPolygon(const VPointCloud::ConstPtr msg,
                                                   ros::NodeHandle priv_nh)
{

  if(msg->points.empty()) return;

  msg_z = msg->makeShared();

  msg_z->is_dense=false;
  pcl::removeNaNFromPointCloud(*msg_z, *msg_z, index);

  for(int i=0; i<msg_z->points.size(); i++) {
    msg_z->points[i].z = 3;
  }

  if(msg_z->points.size() < 10) return;

  pcl::search::KdTree<VPoint>::Ptr tree (new pcl::search::KdTree<VPoint>);
  tree->setInputCloud (msg_z);

  double euc_tolerance = priv_nh.param<double>("obstacle_euclidean_tolerance", 0.18);
  int euc_min_cluster = priv_nh.param<int>("obstacle_euclidean_min_cluster", 50);
  int euc_max_cluster = priv_nh.param<int>("obstacle_euclidean_max_cluster", 50000);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<VPoint>ec;
  ec.setClusterTolerance (euc_tolerance);
  ec.setMinClusterSize (euc_min_cluster);
  ec.setMaxClusterSize (euc_max_cluster);
  ec.setSearchMethod (tree);
  ec.setInputCloud (msg_z);
  ec.extract (cluster_indices);

  arr_polygon_msg_all.polygons.clear();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    VPointCloud::Ptr cloud_cluster (new VPointCloud);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (msg_z->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    cloud_cluster->header.frame_id = "camera_init";

    // Create a Convex Hull representation of the projected inliers
    VPointCloud::Ptr cloud_hull (new VPointCloud);
    pcl::ConvexHull<VPoint> cvhull;
    cvhull.setInputCloud (cloud_cluster);
    cvhull.setDimension(2);
    unique_lock<mutex> lock(mutex_cvxpoly);
    cvhull.reconstruct (*cloud_hull);
    lock.unlock();

    cloud_hull->header.frame_id = "camera_init";

    polygon_msg_all.polygon.points.clear();
    for(int i=0; i<cloud_hull->points.size(); i++) {
      point32_all.x = cloud_hull->points[i].x;
      point32_all.y = cloud_hull->points[i].y;
      point32_all.z = 10;
      polygon_msg_all.polygon.points.push_back(point32_all);
    }
    polygon_msg_all.header.frame_id="camera_init";

    arr_polygon_msg_all.header.frame_id="camera_init";
    arr_polygon_msg_all.polygons.push_back(polygon_msg_all);
    // publish to /all_convex_polygon
    pub_cvx_all.publish(arr_polygon_msg_all);
  }
}

void ConvexPolygonNodelet::points_registered_callback(const VPointCloud::ConstPtr &msg,
                                                      ros::NodeHandle priv_nh)
{

  if(msg->points.empty()) return;

  publishAllConvexPolygon(msg, priv_nh);
}

// void ConvexPolygonNodelet::onInit()
ConvexPolygonNodelet::ConvexPolygonNodelet(ros::NodeHandle nh, ros::NodeHandle priv_nh)
{
  lpc_ptr2 = VPointCloud::Ptr(new VPointCloud);
  msg_sum = VPointCloud::Ptr(new VPointCloud);
  msg_tf = VPointCloud::Ptr(new VPointCloud);
  msg_z = VPointCloud::Ptr(new VPointCloud);

  sub_obs_registered = nh.subscribe<VPointCloud>("points_obstacle_registered", 1, boost::bind(&ConvexPolygonNodelet::points_registered_callback,this, _1, priv_nh));

  pub_cvx_all = nh.advertise<jsk_recognition_msgs::PolygonArray>("all_convex_polygon", 1);
}

} // end of namespace dyrosvehicle
