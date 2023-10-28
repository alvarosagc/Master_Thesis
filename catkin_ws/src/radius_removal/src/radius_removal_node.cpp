#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h> //Este de aqui no creo que lo utilice a priori

class cloudHandler {
		
	public:
	
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;
		
			cloudHandler() {
			//Here we are reading the PointCloud obtained from the pcd file that we are reading
			sub = nh.subscribe("initial_PC2", 10, &cloudHandler::cloud_filtering, this);
			//Creation of the publisher for publishing the PC after the outlier removal
			pub = nh.advertise<sensor_msgs::PointCloud2> ("PC2_radius_removal", 1);
		}
		
		void cloud_filtering (const sensor_msgs::PointCloud2& input) {
			//Creation of the PointCloud objects for the initial cloud and the filtered one
			pcl::PointCloud<pcl::PointXYZ> initial_cloud;
			pcl::PointCloud<pcl::PointXYZ> radius_removal_cloud;
			
			//Creation of the topic output for the expression of the PC 
			sensor_msgs::PointCloud2 output_radius_removal;
			
			pcl::fromROSMsg (input, initial_cloud);
			
			//Creation of the RadiusOutlierRemoval object
			pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_Filter;
			
			//Parametrization of the Radius removal filter
			radius_Filter.setInputCloud(initial_cloud.makeShared());
			radius_Filter.setRadiusSearch(0.1);
			radius_Filter.setMinNeighborsInRadius (6);
			radius_Filter.setKeepOrganized(false);
			
			//Application of the filter
			radius_Filter.filter (radius_removal_cloud);
			
			pcl::toROSMsg(radius_removal_cloud,output_radius_removal);
			pub.publish(output_radius_removal);
			
		}
		
};

main(int argc, char** argv) {
	ros::init(argc, argv, "radius_removal_node");
	
	cloudHandler handler;
	ros::spin();
	
	return 0;
}
			
			