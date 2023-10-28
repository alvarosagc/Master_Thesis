#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
	//Creation of the PointCloud obejct
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::fromROSMsg(input, cloud);

//Command for storing the point cloud in a pcd file 
pcl::io::savePCDFileASCII ("PC1_downsampled.pcd", cloud);
}

main (int argc, char **argv)
{
	
	//Node for the pcd write program
ros::init (argc, argv, "write_pcd_node");

ros::NodeHandle nh;
//Creation of the subscriber
ros::Subscriber sub;

//Subscription to the topic I want to store
sub = nh.subscribe("PC_downsampled_voxel", 10, cloudCB);
ros::spin();

return 0;
}

