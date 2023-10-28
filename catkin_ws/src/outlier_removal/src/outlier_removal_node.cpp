#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

//AVERIGUAR PORQUE HAY QUE CREAR UNA CLASS Y YO CREO QUE EL PUBLIC Y EL PROTECTED NO ME HACE FALTA EN ESTE CASO
class cloudHandler {
		
	public:
	
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;
		
		cloudHandler() {
			//Here we are reading the PointCloud obtained from the pcd file that we are reading
			sub = nh.subscribe("initial_PC2", 10, &cloudHandler::cloud_filtering, this);
			//Creation of the publisher for publishing the PC after the outlier removal
			pub = nh.advertise<sensor_msgs::PointCloud2> ("PC_outlier_removal_statistical", 1);
		}
		
		void cloud_filtering (const sensor_msgs::PointCloud2& input) {
			//Creation of the PointCloud objects for the initial cloud and the filtered one
			pcl::PointCloud<pcl::PointXYZ> initial_cloud;
			pcl::PointCloud<pcl::PointXYZ> outrem_statistical_cloud;
			
			//Creation of the topic output for the expression of the PC 
			sensor_msgs::PointCloud2 output_outrem_statistical;
			
			pcl::fromROSMsg (input, initial_cloud);
			
			//Creation of the StatisticalOutlierRemoval object
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statitistical_Filter;
			
			//Parametrization of the StatisticalOutlierRemoval object
			statitistical_Filter.setInputCloud(initial_cloud.makeShared());
			statitistical_Filter.setMeanK(30);
			statitistical_Filter.setStddevMulThresh(0.001);
			statitistical_Filter.filter(outrem_statistical_cloud);
			
			//Transformation of the PointCloud again to a RosMsg methodoloy
			pcl::toROSMsg(outrem_statistical_cloud, output_outrem_statistical);
			pub.publish(output_outrem_statistical);
		
		}
};
	
	
main (int argc, char** argv) {
	ros::init(argc, argv, "outlier_removal_node");
	
	cloudHandler handler;
	ros::spin();
	
	return 0;
}