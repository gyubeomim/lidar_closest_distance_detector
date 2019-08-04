#include "segmentation.h"

namespace dyrosvehicle {

void SegmentationNodelet::addObstaclePointCloud(VPointCloud& lpc, const VPointCloud& rpc, ros::NodeHandle priv_nh) {
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

void SegmentationNodelet::points_obstacle_filtered_callback(const VPointCloud::ConstPtr &msg,
                                                            tf::TransformListener *listener,
                                                            tf::StampedTransform *transform_,
                                                            ros::NodeHandle priv_nh)
{
  if(msg->points.size() < 10) return;

  jsk_recognition_msgs::PolygonArray arr_polygon_msg;
  VPointCloud::Ptr msg_z(new VPointCloud);

  msg_z = msg->makeShared();

  for(int i=0; i<msg_z->points.size(); i++) {
    msg_z->points[i].z = 3;
  }

  try {
    listener->lookupTransform("odom", "camera", ros::Time(0), *transform_);
  }
  catch (tf::TransformException ex) {
    ROS_WARN("%s", ex.what());
  }

  pcl_ros::transformPointCloud(*msg_z, *msg_tf, *transform_);
  addObstaclePointCloud(*msg_sum, *msg_tf, priv_nh);

  double voxel_leaf = priv_nh.param<double>("obstacle_voxel_leaf_size", 0.2);

  pcl::VoxelGrid<VPoint> vg;
  vg.setInputCloud(msg_sum);
  vg.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
  vg.filter(*msg_sum_vg);

  tf::Transform inv_transform = transform_->inverse();
  pcl_ros::transformPointCloud(*msg_sum_vg, *msg_sum_tf, inv_transform);

  vector<int> index;
  msg_sum_tf->is_dense=false;
  pcl::removeNaNFromPointCloud(*msg_sum_tf, *msg_sum_tf_removenan, index);

  msg_sum_tf_removenan->header.frame_id="camera_init";

  pcl::search::KdTree<VPoint>::Ptr tree (new pcl::search::KdTree<VPoint>);
  tree->setInputCloud (msg_sum_tf_removenan);

  double euc_tolerance = priv_nh.param<double>("obstacle_euclidean_tolerance", 0.13);
  int euc_min_cluster = priv_nh.param<int>("obstacle_euclidean_min_cluster", 50);
  int euc_max_cluster = priv_nh.param<int>("obstacle_euclidean_max_cluster", 50000);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<VPoint>ec;
  ec.setClusterTolerance (euc_tolerance);
  ec.setMinClusterSize (euc_min_cluster);
  ec.setMaxClusterSize (euc_max_cluster);
  ec.setSearchMethod (tree);
  ec.setInputCloud (msg_sum_tf_removenan);
  ec.extract (cluster_indices);

  arr_polygon_msg.polygons.clear();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    VPointCloud::Ptr cloud_cluster (new VPointCloud);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (msg_sum_tf_removenan->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Create a Convex Hull representation of the projected inliers
    VPointCloud::Ptr cloud_hull (new VPointCloud);
    pcl::ConvexHull<VPoint> cvhull;

    cvhull.setInputCloud (cloud_cluster);
    cvhull.setDimension(2);

    try {
      unique_lock<mutex> lock(mutex_seg);
      cvhull.reconstruct (*cloud_hull);
      lock.unlock();
    }
    catch (std::exception e){
      cout << e.what() << endl;
      return;
    }

    geometry_msgs::Point32 point32;
    geometry_msgs::PolygonStamped polygon_msg;
    polygon_msg.polygon.points.clear();
    for(int i=0; i<cloud_hull->points.size(); i++) {
      point32.x = cloud_hull->points[i].x;
      point32.y = cloud_hull->points[i].y;
      point32.z = 10;
      polygon_msg.polygon.points.push_back(point32);
    }
    polygon_msg.header.frame_id="camera_init";

    if(polygon_msg.polygon.points.size() == 0) continue;

    arr_polygon_msg.header.frame_id="camera_init";
    arr_polygon_msg.header.stamp = ros::Time::now();
    arr_polygon_msg.polygons.push_back(polygon_msg);
    // publish to /current_convex_polygon
    pub_cvx_current.publish(arr_polygon_msg);
  }
}

void SegmentationNodelet::timer_callback(const ros::TimerEvent &event) {
  msg_sum->points.clear();
}

void SegmentationNodelet::onInit() {
  nh = getNodeHandle();
  priv_nh = getPrivateNodeHandle();

  msg_tf = VPointCloud::Ptr(new VPointCloud);
  msg_sum = VPointCloud::Ptr(new VPointCloud);
  msg_sum_vg = VPointCloud::Ptr(new VPointCloud);
  msg_sum_tf = VPointCloud::Ptr(new VPointCloud);
  msg_sum_tf_removenan = VPointCloud::Ptr(new VPointCloud);

  sub_obs_filtered = nh.subscribe<VPointCloud>("points_obstacle_filtered", 1, boost::bind(&SegmentationNodelet::points_obstacle_filtered_callback, this, _1, &listener, &transform_, priv_nh));

  pub_cvx_current = nh.advertise<jsk_recognition_msgs::PolygonArray>("current_convex_polygon", 1);

  timer = nh.createTimer(ros::Duration(10), &SegmentationNodelet::timer_callback, this);
}

} // namespace dyrosvehicle
