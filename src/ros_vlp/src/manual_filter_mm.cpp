#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/point_picking_event.h>

int main(int argc, char** argv){
	std::string fname;
	if(argc==2){
		std::stringstream ss;
		ss<<argv[1];
		fname=ss.str();
		std::cout<<fname<<" is opened"<<std::endl;
	}else{
		std::cout<<"Please input PCD file name"<<std::endl;
		return 0;
	}
	//initVis();
	sensor_msgs::PointCloud2 PointCloud;
	typedef pcl::PointXYZI PointType;
	//pcl::PointCloud<PointType> trcloud;
	pcl::PointCloud<PointType>::Ptr trcloud2(new pcl::PointCloud<PointType>);

	//use pcl viewer
	// PCL Visualizer
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "align Viewer" ) );
	// Point Cloud Color Hndler
	//pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
	//Point cloud Vector
	pcl::PointCloud<PointType>::Ptr vizClouds(new pcl::PointCloud<PointType>);
	//pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

	// PCL Visualizer
    	/*viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
	viewer->addCoordinateSystem( 500.0  ,"coordinate" );
    	viewer->initCameraParameters();
    	viewer->setCameraPosition( 500.0, 0.0, 8000.0, 0.0, 0.0, 0.75, 0 );

        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType> > color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;*/
	
	ros::init(argc, argv, "manual_visualizer");
	ros::NodeHandle nh;

	//visualizer();
	//Point cloud Vector
	pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
	//std::printf(" Visualser\n");
 
	pcl::io::loadPCDFile(fname,*vizClouds);
	
        //create filtering object
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud (vizClouds);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (500,20000); //2600
        //pass.setFilterLimitsNegative(true);
        pass.filter (*filtered_cloud);

        // Update Point Cloud (Viewewr)
	/*handler->setInputCloud(filtered_cloud);
        if( !viewer->updatePointCloud( filtered_cloud, *handler, "trcloud" ) ){
		viewer->addPointCloud( filtered_cloud, *handler, "trcloud" );
		}*/

	std::string save_fname;
	std::stringstream ss1;
	ss1<<"filtered/filtered_"<<argv[1];
	save_fname=ss1.str();
	pcl::io::savePCDFileBinary(save_fname,*filtered_cloud);
	std::cout<<"filterd file is saved : "<<save_fname<<std::endl;

	// Update Viewer
        //viewer->spin();
	
	//ros::spin();
}
