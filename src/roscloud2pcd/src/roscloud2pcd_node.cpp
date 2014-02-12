/*
 * roscloud2pcd_node.cpp
 * Author: Vittorio
 *  Notes: This program will wait for 5 frames, and save the fifth frame to a pcd file
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h> // DEPRECATED
#include <pcl/io/pcd_io.h>


class roscloud2pcd
{

public:
	int cloudCounter;
	// Constructor
	roscloud2pcd(): viewer ("roscloud2pcd")
	{
		cloudCounter = 0;

		ROS_INFO("Entering Constructor!!!");
		rototranslatedpcSubscriber.subscribe(node, "/rototranslatedpc", 5);
		rototranslatedpcSubscriber.registerCallback(boost::bind(&roscloud2pcd::cloud_cb, this, _1));
		//pclCloudBoostPtr (new pcl::PointCloud<pcl::PointXYZ> );
	}

	// DECLARATIONS
	ros::NodeHandle node; // Node name is "node"
	message_filters::Subscriber<sensor_msgs::PointCloud2> rototranslatedpcSubscriber;
	pcl::visualization::CloudViewer viewer;

	void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& rototranslatedpcBoostPtr){
		 boost::shared_ptr <pcl::PointCloud <pcl::PointXYZ> > pclCloudBoostPtr (new pcl::PointCloud<pcl::PointXYZ> );


		if (!viewer.wasStopped())  viewer.showCloud (pclCloudBoostPtr, "This is the cloud that was saved!"); // IGNORE ECLIPSE ERROR HERE. COMPILER WORKS.
		cloudCounter++;
		if ( cloudCounter == 5 ) {

			pcl::fromROSMsg( *rototranslatedpcBoostPtr , *pclCloudBoostPtr );
			pcl::io::savePCDFileASCII ("captured_roscloud.pcd", *pclCloudBoostPtr);
	  		std::cerr << "Saved " << pclCloudBoostPtr->points.size () << " data points to test_pcd.pcd." << std::endl;

		}
	}
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "roscloud2pcd"); // Init ROS
	roscloud2pcd roscloud2pcd_OBJECT; // Instance Object
	ros::spin(); // Run until interrupted
}
