#include <ros/ros.h>
#include <iostream>
// PCL specific includes
 #include <pcl_conversions/pcl_conversions.h>
 #include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
{

  //pcl::PCLPointCloud2 cloud_filtered;
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;

    //pass through filter
    pass_through_filter.setInputCloud (input);
    pass_through_filter.filter (*cloud_pass);

    //voxel grid filter
    sor.setInputCloud(cloud_pass);
    sor.setLeafSize(0.05, 0.05, 0.05);
    sor.filter(*cloud_filtered);


    // Publish the data
    pub.publish(*cloud_filtered);
}

int main (int argc, char** argv){
// Initialize ROS
ros::init (argc, argv, "sabir");
ros::NodeHandle nh;

 // Create a ROS subscriber for the input point cloud
ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

// Create a ROS publisher for the output point cloud
pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

// Spin
ros::spin ();
}
