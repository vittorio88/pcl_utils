/*
 * pc_viewer_ros_node.cpp
 * Author: Vittorio
 *  Notes: This program visualizes a pointcloud from ROS using the pointcloud visualizer from the pcl.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>


class pc_viewer_ros
{

public:

	// Constructor
	pc_viewer_ros(): viewer ("PCL Viewer")
	{
		ROS_INFO("Entering Constructor!!!");
		rototranslatedpcSubscriber.subscribe(node, "/rototranslatedpc", 5);
		rototranslatedpcSubscriber.registerCallback(boost::bind(&pc_viewer_ros::cloud_cb, this, _1));
	}

	// DECLARATIONS
	ros::NodeHandle node; // Node name is "node"
	message_filters::Subscriber<sensor_msgs::PointCloud2> rototranslatedpcSubscriber;
	pcl::visualization::CloudViewer viewer;

	void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& rototranslatedpcBoostPtr){
		boost::shared_ptr <pcl::PointCloud <pcl::PointXYZ> > pclCloudBoostPtr (new pcl::PointCloud<pcl::PointXYZ> );


		pcl::fromROSMsg( *rototranslatedpcBoostPtr , *pclCloudBoostPtr ); // ORIG WORKING

		//		// Perform voxel filter
		//		boost::shared_ptr <pcl::PointCloud <pcl::PointXYZ> > filteredCloudBoostPtr (new pcl::PointCloud<pcl::PointXYZ> ); // Uncomment to use filtering
		//		pcl::VoxelGrid<pcl::PointXYZ> sor;
		//		  sor.setInputCloud (pclCloudBoostPtr);
		//		  sor.setLeafSize (0.01f, 0.01f, 0.01f);
		//		  sor.filter (*filteredCloudBoostPtr);
		//
		//		// Prints filtered pointcloud
		//		for (size_t i = 0; i < filteredCloudBoostPtr->points.size (); ++i)
		//			std::cout << filteredCloudBoostPtr->points[i].x
		//			<< "     "<< filteredCloudBoostPtr->points[i].y
		//			<< "     "<< filteredCloudBoostPtr->points[i].z << std::endl;
		//
		//		  std::cout<<filteredCloudBoostPtr->points.size ()<<std::endl;
		// if (!viewer.wasStopped())	viewer.showCloud (filteredCloudBoostPtr, "sample cloud");
		if (!viewer.wasStopped())	viewer.showCloud ( pclCloudBoostPtr, "sample cloud"); // IGNORE ECLIPSE ERROR HERE. COMPILER WORKS.

	}
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "pc_viewer_ros"); // Init ROS
	pc_viewer_ros pc_viewer_ros_OBJECT; // Instance Object
	ros::spin(); // Run until interrupted
}
