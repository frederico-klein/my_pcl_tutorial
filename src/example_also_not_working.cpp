#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl_ros/filters/passthrough.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
//cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  const pcl_msgs::PointIndicesConstPtr indices;
  // Do data processing here...
  pcl_ros::PassThrough::filter (input, indices , output);
  output = *input;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);



  // Spin
  ros::spin ();
}
