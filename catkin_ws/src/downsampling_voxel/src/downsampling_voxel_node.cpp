#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

class cloudHandler {
	
	public: 
	 
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;
		
		cloudHandler(){
			sub = nh.subscribe("initial_PC2",10, &cloudHandler::cloudCB, this);
			pub = nh.advertise<sensor_msgs::PointCloud2> ("PC_downsampled_voxel",1);
		}
		
		void cloudCB (const sensor_msgs::PointCloud2& input){
			
			pcl::PointCloud<pcl::PointXYZ> outlier_cloud;
			pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
			
			sensor_msgs::PointCloud2 output_downsampling_voxel;
			
			pcl::fromROSMsg(input, outlier_cloud);
			
			pcl::VoxelGrid<pcl::PointXYZ> voxel_downsampling;
			
			voxel_downsampling.setInputCloud (outlier_cloud.makeShared());
			voxel_downsampling.setLeafSize(0.01f, 0.01f, 0.01f);
			voxel_downsampling.filter (downsampled_cloud);
			
			pcl::toROSMsg (downsampled_cloud, output_downsampling_voxel);
			pub.publish (output_downsampling_voxel);
		}
};

main (int argc, char **argv) {
	
	ros::init(argc, argv, "downsampling_voxel_node");
	
	cloudHandler handler;
	ros::spin();
	
	return 0;
}