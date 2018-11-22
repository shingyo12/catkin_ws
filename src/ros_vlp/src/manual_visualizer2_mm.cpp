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
#include <pcl/point_cloud.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/point_picking_event.h>

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void){
	static int cb_cnt=0;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	float x,y,z;
	if(event.getPointIndex() == -1){
		return;
	}
        event.getPoint(x,y,z);
	//std::cout<<"point"<<cb_cnt<<" ("<<x<<","<<y<<","<<z<<")"<<std::endl;
	std::cout<<x<<" "<<y<<" "<<z<<std::endl;
	cb_cnt++;
}

void pp_callback2(const pcl::visualization::AreaPickingEvent& event,void* viewer_void){
	std::cout<<"callback2"<<std::endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	std::vector<int> indices;
	if(event.getPointsIndices(indices) == -1){
		std::cout<<indices.size()<<std::endl;
	}
}

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
	typedef pcl::PointXYZRGB PointType;
	//pcl::PointCloud<PointType> trcloud;
	pcl::PointCloud<PointType>::Ptr trcloud2(new pcl::PointCloud<PointType>);

	//use pcl viewer
	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "align Viewer" ) );
	// Point Cloud Color Hndler
	pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
	//Point cloud Vector
	pcl::PointCloud<PointType>::Ptr vizClouds(new pcl::PointCloud<PointType>);
	//pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

	// PCL Visualizer
    	viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
	viewer->addCoordinateSystem( 2500.0  ,"coordinate" );
    	viewer->initCameraParameters();
    	viewer->setCameraPosition( 500.0, 0.0, 80000.0, 0.0, 0.0, 0.0, 0 );

        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType> > color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType> ("rgb") );
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(vizClouds);
        handler = color_handler;

	viewer->registerPointPickingCallback(pp_callback, (void*)&viewer);
	viewer->registerAreaPickingCallback(pp_callback2, (void*)&viewer);
 	
	ros::init(argc, argv, "manual_visualizer");
	ros::NodeHandle nh;

	//visualizer();
	//Point cloud Vector
	//pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
	//pcl::PointCloud<PointType>::Ptr filtered_cloud2(new pcl::PointCloud<PointType>);
	//std::printf(" Visualser\n");

	//pcl::PointCloud<PointType>::Ptr vizClouds(new pcl::PointCloud<PointType>);
	pcl::io::loadPCDFile(fname,*vizClouds);

	std::cout<<"Shift+click two points"<<std::endl;

        // Update Point Cloud (Viewewr)
	handler->setInputCloud(vizClouds);
        if( !viewer->updatePointCloud( vizClouds,  "trcloud" ) ){
		viewer->addPointCloud( vizClouds,  "trcloud" );
	}
	

	// Update Viewer
        viewer->spin();
	
	//ros::spin();
}
