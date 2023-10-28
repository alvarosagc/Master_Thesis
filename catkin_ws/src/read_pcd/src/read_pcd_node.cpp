#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

main (int argc, char **argv) {
	
	//Creation of the Rosnode used to read the .pcd document in ROS
	ros::init (argc, argv, "read_pcd_node");
	
	ros::NodeHandle nh;
	
	//Creation of the publisher
	ros::Publisher pub = nh.advertise <sensor_msgs::PointCloud2> ("initial_PC2", 1);
	//Creation of the PointCloud2 message
	sensor_msgs::PointCloud2 output;
	//Creation of an PointCloud object for storing there the point cloud from the .pcd file
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	
	pcl::io::loadPCDFile ("scene_filename_2.pcd", point_cloud);
	pcl::toROSMsg (point_cloud, output);
	
	//LA FRAME DE CREATION QUE TIENE ES USANDO EL TOPICO ODOM, NO SE SI ESTI SERA TAMBIEN CORRECTO EN NUESTRO PROYECTO
	output.header.frame_id = "odom";
	
	ros::Rate loop_rate(1);
	while (ros::ok()) {
		pub.publish(output);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
return 0;	
		
}