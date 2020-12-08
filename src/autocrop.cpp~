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
#include <my_pcl_tutorial/AutoCropConfig.h>

#include <geometry_msgs/Point.h>

ros::Publisher pub;

double_t LEN_X, LEN_Y, LEN_Z;
double_t fpointx, fpointy, fpointz;
double_t gainX, gainY, gainZ;
double_t centreX, centreY, centreZ;

void callback(my_pcl_tutorial::AutoCropConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: LEN (%f %f %f)\n gain (%f %f %f)\n centre LEN (%f %f %f)",
            config.LEN_X,
            config.LEN_Y,
            config.LEN_Z,
            config.gain_X,
            config.gain_Y,
            config.gain_Z,
            config.centre_X,
            config.centre_Y,
            config.centre_Z);
            LEN_X = config.LEN_X;
            LEN_Y = config.LEN_Y;
            LEN_Z = config.LEN_Z;
            gainX = config.gain_X;
            gainY = config.gain_Y;
            gainZ = config.gain_Z;
            centreX = config.centre_X;
            centreY = config.centre_Y;
            centreZ = config.centre_Z;
}

void update_point(geometry_msgs::Point funny_point)
{
//idk what to do with the size
//also, this is in pixels. there is the need for a good transformation here. i will bodge somehting
fpointx = funny_point.x ;
fpointy = funny_point.y ;
fpointz = funny_point.z ;
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
{
  double_t pointx = (fpointx - centreX)/gainX;
  double_t pointy = (fpointy - centreY)/gainY;
  double_t size =   (fpointz - centreZ)/gainZ;
  //pcl::PCLPointCloud2 cloud_filtered;
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr box_out(new pcl::PCLPointCloud2);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;

    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud(input);
    Eigen::Vector4f min_point = Eigen::Vector4f(pointx- LEN_X, pointy -LEN_Y, size -LEN_Z, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(pointx+ LEN_X, pointy +LEN_Y, size +LEN_Z, 0);
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
ros::init (argc, argv, "AutoCropBox");
ros::NodeHandle nh;

 // Create a ROS subscriber for the input point cloud
ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

// Create a ROS publisher for the output point cloud
pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

//now the dynamic_reconfigure server stuff
dynamic_reconfigure::Server<my_pcl_tutorial::AutoCropConfig> server;
dynamic_reconfigure::Server<my_pcl_tutorial::AutoCropConfig>::CallbackType f;

f = boost::bind(&callback, _1, _2);
server.setCallback(f);

//read my special point message:
geometry_msgs::Point pose_msgs;
ros::Subscriber funny_point = nh.subscribe("funny_point", 1, update_point);

// Spin
ros::spin ();
}
