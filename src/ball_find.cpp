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

//segmentation stuff
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

//dynamic_reconfigure stuff
#include <dynamic_reconfigure/server.h>
#include <my_pcl_tutorial/BallConfig.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub, pub_intermediate, ball_marker,topo_point, vis_pub;

visualization_msgs::Marker marker;

double_t LEN_X, LEN_Y, LEN_Z;
double_t fpointx, fpointy, fpointz;
double_t gainX, gainY, gainZ;
double_t centreX, centreY, centreZ;
double_t ball_radius;


void callback(my_pcl_tutorial::BallConfig &config, uint32_t level) {
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
            ball_radius = config.ball_r;
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
  pcl::PCLPointCloud2::Ptr bola_box_out(new pcl::PCLPointCloud2);

  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZ> ());  
  //pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentation_from_normals;
  //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;

    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud(input);
    Eigen::Vector4f min_point = Eigen::Vector4f(pointx- LEN_X, pointy -LEN_Y, size -LEN_Z, 1);
    Eigen::Vector4f max_point = Eigen::Vector4f(pointx+ LEN_X, pointy +LEN_Y, size +LEN_Z, 1);
    crop.setMin(min_point);
    crop.setMax(max_point);

    crop.filter(*box_out);

    pub_intermediate.publish(*box_out);

    //find a ball!
    //the top is stable. 
    //lazy solution. find the topmost point
    pcl::PointXYZ toppopointo;
    toppopointo.x = 0;
    toppopointo.y = 10;
    toppopointo.z = 0;

    pcl::PointCloud<pcl::PointXYZ> another_normal_cloud; //what I should be doing all along!
    fromPCLPointCloud2 (*box_out, another_normal_cloud);

      for (auto& point: another_normal_cloud)
        {
          if (toppopointo.y > point.y) //I think y is inverted.
          {
            toppopointo.x = point.x;
            toppopointo.y = point.y;
            toppopointo.z = point.z;
          } 
        }

    geometry_msgs::Point topo_pointo;
    topo_pointo.x = toppopointo.x;
    topo_pointo.y = toppopointo.y;
    topo_pointo.z = toppopointo.z;

    topo_point.publish(topo_pointo);
    ///i am doing a transformation wrong. no time. 
    marker.pose.position.x = toppopointo.z;
    marker.pose.position.y = -toppopointo.x;
    marker.pose.position.z = -toppopointo.y;
    marker.header.stamp = ros::Time();

    vis_pub.publish( marker );

      //now I should have the top point. lets create a new box

      pcl::PointXYZ centerbola;
      centerbola = toppopointo;
      double_t radiusbola = ball_radius ; //whats the unit? assuming metric
      centerbola.y = centerbola.y - radiusbola; 
      pcl::CropBox<pcl::PCLPointCloud2> around_the_ball_box;

    around_the_ball_box.setInputCloud(box_out);
    Eigen::Vector4f min_point_aroundthebox = Eigen::Vector4f(centerbola.x - radiusbola, centerbola.y -radiusbola, centerbola.z -radiusbola, 1); //x is inverted?
    Eigen::Vector4f max_point_aroundthebox = Eigen::Vector4f(centerbola.x + radiusbola, centerbola.y +radiusbola, centerbola.z +radiusbola, 1);
    around_the_ball_box.setMin(min_point_aroundthebox);
    around_the_ball_box.setMax(max_point_aroundthebox);

    around_the_ball_box.filter(*bola_box_out);

    // Publish the data
    pub.publish(*bola_box_out);

    pcl::PointCloud<pcl::PointXYZ> normal_cloud; //what I should be doing all along!
    fromPCLPointCloud2 (*bola_box_out, normal_cloud);

    Eigen::Vector4f centroid; 
    pcl::compute3DCentroid (normal_cloud, centroid); 
    ROS_DEBUG_STREAM( "centroid:" << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n");

    geometry_msgs::Point better_point;
    better_point.x = centroid[0];
    better_point.y = centroid[1];
    better_point.z = centroid[2];

    ball_marker.publish(better_point);

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
ros::init (argc, argv, "BallAutoCropBox");
ros::NodeHandle nh;

 // Create a ROS subscriber for the input point cloud
ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

// Create a ROS publisher for the output point cloud
pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

pub_intermediate = nh.advertise<sensor_msgs::PointCloud2> ("output_intermediate", 1);


//now the dynamic_reconfigure server stuff
dynamic_reconfigure::Server<my_pcl_tutorial::BallConfig> server;
dynamic_reconfigure::Server<my_pcl_tutorial::BallConfig>::CallbackType f;

f = boost::bind(&callback, _1, _2);
server.setCallback(f);

//read my special point message:
geometry_msgs::Point pose_msgs;
ros::Subscriber funny_point = nh.subscribe("funny_point", 1, update_point);
ball_marker = nh.advertise<geometry_msgs::Point>("better_point", 1);

topo_point = nh.advertise<geometry_msgs::Point>("topo_point", 1);
vis_pub = nh.advertise<visualization_msgs::Marker>( "toppo_marker", 0 );


marker.header.frame_id = "camera_link";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id = 0;
marker.type = visualization_msgs::Marker::SPHERE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = 1;
marker.pose.position.y = 1;
marker.pose.position.z = 1;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 0.01;
marker.scale.y = 0.01;
marker.scale.z = 0.01;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";



// Spin
ros::spin ();
}
