#include <ros/ros.h>
#include <iostream>
// PCL specific includes
 #include <pcl_conversions/pcl_conversions.h>
 #include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

//dynamic_reconfigure stuff
#include <dynamic_reconfigure/server.h>
#include <my_pcl_tutorial/CropBoxConfig.h>

ros::Publisher pub;

Eigen::Vector4f min_point = Eigen::Vector4f(-1.0, -1.0, -1.0, 0);
Eigen::Vector4f max_point = Eigen::Vector4f(1.0, 1.0, 1.0, 0);

void callback(my_pcl_tutorial::CropBoxConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: MIN (%f %f %f) MAX (%f %f %f)",
            config.MIN_X,
            config.MIN_Y,
            config.MIN_Z,
            config.MAX_X,
            config.MAX_Y,
            config.MAX_Z);
            min_point = Eigen::Vector4f(config.MIN_X, config.MIN_Y, config.MIN_Z, 0);
            max_point = Eigen::Vector4f(config.MAX_X, config.MAX_Y, config.MAX_Z, 0);

/*
ROS_INFO("Reconfigure Request: %d %f %s %s %d",
          config.int_param, config.double_param,
          config.str_param.c_str(),
          config.bool_param?"True":"False",
          config.size);

*/


}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
{

  //pcl::PCLPointCloud2 cloud_filtered;
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr box_out(new pcl::PCLPointCloud2);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;

    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud(input);
    //Eigen::Vector4f min_point = Eigen::Vector4f(-1.0, -1.0, -1.0, 0);
    //Eigen::Vector4f max_point = Eigen::Vector4f(1.0, 1.0, 1.0, 0);
    crop.setMin(min_point);
    crop.setMax(max_point);

    crop.filter(*box_out);

    // Publish the data
    pub.publish(*box_out);

    /*
    //pass through filter
    //pass_through_filter.setInputCloud (input);
    pass_through_filter.filter (*cloud_pass);

    //voxel grid filter
    sor.setInputCloud(cloud_pass);
    sor.setLeafSize(0.05, 0.05, 0.05);
    sor.filter(*cloud_filtered);


    // Publish the data
    pub.publish(*cloud_filtered);

    */
}

int main (int argc, char** argv){
// Initialize ROS
ros::init (argc, argv, "CropBox");
ros::NodeHandle nh;

 // Create a ROS subscriber for the input point cloud
ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

// Create a ROS publisher for the output point cloud
pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

//now the dynamic_reconfigure server stuff
dynamic_reconfigure::Server<my_pcl_tutorial::CropBoxConfig> server;
dynamic_reconfigure::Server<my_pcl_tutorial::CropBoxConfig>::CallbackType f;

f = boost::bind(&callback, _1, _2);
server.setCallback(f);

// Spin
ros::spin ();
}
