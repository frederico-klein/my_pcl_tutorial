//#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// Create the filtering object
pcl::PassThrough<sensor_msgs::PointCloud2> pass;



ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  //output = *input;
  pass.setInputCloud (input);

  //pass.setFilterLimitsNegative (true);
  pass.filter (output);

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

  //set filter parameters

  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);

  // Spin
  ros::spin ();
}
