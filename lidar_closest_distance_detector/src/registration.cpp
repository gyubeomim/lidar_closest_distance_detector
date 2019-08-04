#include "registration.h"

namespace dyrosvehicle {

void RegistrationNodelet::addObstaclePointCloud(VPointCloud& lpc, const VPointCloud& rpc) {
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

  // comment(edward): fix obstacle_points_registered issue which pointcloud goes to sparser
  // if (rpc.is_dense && lpc.is_dense)
  //   lpc.is_dense = true;
  // else
  //   lpc.is_dense = false;
  lpc.is_dense = false;

  lpc_ptr2 = lpc.makeShared();
  pcl::VoxelGrid<VPoint> vg;
  vg.setInputCloud(lpc_ptr2);
  vg.setLeafSize(0.2,0.2,0.2);
  vg.filter(lpc);
}

void RegistrationNodelet::points_obstacle_filtered_callback(const VPointCloud::ConstPtr &msg,
                                                            tf::TransformListener *listener,
                                                            tf::StampedTransform *transform_)
{
  try {
    listener->lookupTransform("odom", "camera", ros::Time(0), *transform_);
  }
  catch (tf::TransformException ex) {
    ROS_WARN("%s", ex.what());
  }

  pcl_ros::transformPointCloud(*msg, *msg_tf, *transform_);

  addObstaclePointCloud(*msg_sum, *msg_tf);

  msg_sum->header.frame_id = "odom";
  // publish to /points_obstacle_registered
  pub_obstacle_registered.publish(msg_sum);
}

void RegistrationNodelet::spacefilter_callback(const VPointCloud::ConstPtr &msg,
                                               ros::NodeHandle priv_nh) {
  if(loam_toggle == false) return;

  if(msg->points.empty()) return;

  double lower = priv_nh.param<double>("spacefilter_lower_limit", -1.0);
  double upper = priv_nh.param<double>("spacefilter_upper_limit", 3.0);
  double voxel_leaf = priv_nh.param<double>("spacefilter_voxel_leaf_size", 0.2);

  pcl::PassThrough<VPoint> ptfilter(true);
  ptfilter.setInputCloud (msg);
  ptfilter.setFilterFieldName ("x");
  ptfilter.setFilterLimits (lower, upper);
  ptfilter.setNegative (false);
  ptfilter.filter (*msg_ptfiltered);

  pcl::VoxelGrid<VPoint> vg;
  vg.setInputCloud(msg_ptfiltered);
  vg.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
  vg.filter(*msg_vg);

  msg_vg->header.frame_id = "odom";
  // publish to /points_obstacle
  pub_obstacle.publish(msg_vg);
}

void RegistrationNodelet::velodyne_callback(const VPointCloud::ConstPtr &scan)
{
  if(loam_toggle == false) return;

  velodyne_frame = scan->header.frame_id.c_str();

  VPointCloud::Ptr cloud_filtered(new VPointCloud);
  VPointCloud::Ptr cloud_filtered2(new VPointCloud);
  VPointCloud::Ptr cloud_out(new VPointCloud);

  if(scan->points.size() == 0) return;

  pcl::PassThrough<VPoint> ptfilter(true);
  ptfilter.setInputCloud (scan);
  ptfilter.setFilterFieldName ("x");
  ptfilter.setFilterLimits (-9.5, 9.5);
  ptfilter.setNegative (false);
  ptfilter.filter (*cloud_filtered);

  ptfilter.setInputCloud(cloud_filtered);
  ptfilter.setFilterFieldName ("y");
  ptfilter.setFilterLimits (-6.6, 6.6);
  ptfilter.setNegative (false);
  ptfilter.filter (*cloud_filtered2);

  ptfilter.setInputCloud(cloud_filtered2);
  ptfilter.setFilterFieldName ("z");
  ptfilter.setFilterLimits (-5, -0.5);
  ptfilter.setNegative (false);
  ptfilter.filter (*cloud_out);

  cloud_out->header.frame_id = "camera";
  // publish to /velodyne_points_modified
  pub_velodyne.publish(cloud_out);
}

void RegistrationNodelet::loam_toggle_callback(const std_msgs::Bool::ConstPtr &msg) {
  loam_toggle = msg->data;
}

void RegistrationNodelet::onInit() {
  nh = getNodeHandle();
  priv_nh = getPrivateNodeHandle();

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  msg_tf = VPointCloud::Ptr(new VPointCloud);
  msg_ptfiltered = VPointCloud::Ptr(new VPointCloud);
  msg_vg = VPointCloud::Ptr(new VPointCloud);
  msg_sum = VPointCloud::Ptr(new VPointCloud);
  lpc_ptr2 = VPointCloud::Ptr(new VPointCloud);

  loam_toggle = false;

  // Subscribers
  sub_velodyne = nh.subscribe("points_raw", 1, &RegistrationNodelet::velodyne_callback, this);
  sub_spacefilter = nh.subscribe<VPointCloud>("points_clipped", 1, boost::bind(&RegistrationNodelet::spacefilter_callback, this, _1, priv_nh));
  sub_obstacle_filtered = nh.subscribe<VPointCloud>("points_obstacle_filtered", 1, boost::bind(&RegistrationNodelet::points_obstacle_filtered_callback, this, _1 ,&listener, &transform_));

  sub_loam_toggle = nh.subscribe<std_msgs::Bool>("loam_toggle", 1, &RegistrationNodelet::loam_toggle_callback, this);

  // Publishers
  pub_velodyne = nh.advertise<VPointCloud>("velodyne_points_modified", 1);
  pub_obstacle = nh.advertise<VPointCloud>("points_obstacle", 1);
  pub_obstacle_registered = nh.advertise<VPointCloud>("points_obstacle_registered", 1);
}
} // namespace dyrosvehicle
