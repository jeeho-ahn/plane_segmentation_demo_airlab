/*
 * AIR Lab Demo ROS node for plane extraction
 * Extract and find floor plane with similar orientation to a given reference vector using RANSAC and get it's normal vector
 * 2021.3.2
 * Written by Ahn, Jeeho
*/

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/service.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

//common
#include <pwd.h>
#include <fstream>
#include <string>

//eigen
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

#include "floor_extraction.h"

using namespace std;

ros::Publisher* pubPlanePcdPtr;

void handle_pcd(const sensor_msgs::PointCloud2::ConstPtr& pcd_in)
{
  //convert sensormsg to pcl
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pcd_in,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  //ransac single plane
  auto seg = extract_floor(temp_cloud,true);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 out_cloud;
  pcl::toROSMsg(*seg.first,out_cloud);
  // Publish the data
  pubPlanePcdPtr->publish (out_cloud);
}

int main(int argc, char** argv)
{
  // ros initialization
  std::cout << "\tFloor Extraction Demo" << std::endl;
  std::cout << "\tReference Floor Vector: [0, 0, 1]" << std::endl;

  ros::init(argc, argv, "floor_extraction_node");

  ros::NodeHandle nh;
  //ros::Rate rate(200); //200 hz
  //ros::Duration(1.0).sleep(); //for debug attach

  //subscribe velodyne
  ros::Subscriber subRealsense = nh.subscribe<sensor_msgs::PointCloud2>("/sample_pcd",10,handle_pcd);
  //publish plane pcd
  ros::Publisher pubPlanePcd = nh.advertise<sensor_msgs::PointCloud2>("/plane_pcd", 1);
  pubPlanePcdPtr = &pubPlanePcd;

  while(ros::ok())
  {
    //ros::spinOnce();
    //rate.sleep();
    ros::spin();
  }

  return 0;
}
