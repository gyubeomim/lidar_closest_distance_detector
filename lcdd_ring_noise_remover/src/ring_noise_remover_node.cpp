/*
 * #+DESCRIPTION: Ring Noise Removal Filter
 
 * #+DATE:        2018-12-20 Thu
 * #+AUTHOR:      Edward Im (gyurse@gmail.com)
 */
#define PCL_NO_PRECOMPILE
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

using namespace std;

namespace pcl {
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace pcl


POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))
PCL_INSTANTIATE_PointCloud(pcl::PointXYZIR);

ros::Publisher velo_pub;

typedef pcl::PointXYZIR PointXYZIR;
typedef pcl::PointCloud<PointXYZIR> PointCloudXYZIR;

void velo_callback(const PointCloudXYZIR::ConstPtr& msg){
  PointCloudXYZIR::Ptr cloud_filtered(new PointCloudXYZIR);

  cloud_filtered->header.frame_id = msg->header.frame_id;
  cloud_filtered->header.stamp = msg->header.stamp;
  for(int i=0; i<=msg->points.size()-1; i++) {
    double x = msg->points[i].x;
    double y = msg->points[i].y;
    double z = msg->points[i].z;
    double dist = sqrt(x*x + y*y);

    // comment(edward): centering removal filter
    if(dist < 1.8 && z > -0.5)   continue;
    else if (z > -0.2) continue;
    else {
      cloud_filtered->points.push_back(msg->points[i]);
    }
  }
  // publish to /points_raw
  velo_pub.publish(cloud_filtered);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ps_ring_noise_remover_node");
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber velo_sub = nh.subscribe("/velodyne_points", 1, velo_callback);

  // Publishers
  velo_pub = nh.advertise<sensor_msgs::PointCloud2>("points_raw", 1);

  cout << "[+] ring_noise_remover_node for parkingSLAM has started..." << endl;

  ros::spin();
  return 0;
}
