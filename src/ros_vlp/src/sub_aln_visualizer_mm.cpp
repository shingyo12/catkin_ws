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
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>

sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointXYZI PointType;
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
int total=0,j=0;

void initVis(){
	// PCL Visualizer
    	viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
	viewer->addCoordinateSystem( 5.0  ,"coordinate" );
    	viewer->initCameraParameters();
    	viewer->setCameraPosition( 25000.0, 0.0, 30000.0, 0.0, 0.0, 0.0, 0 );

        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType> > color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;
}

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	float x,y,z;
	if(event.getPointIndex() == -1){
		return;
	}
        event.getPoint(x,y,z);
	std::cout<< "("<<x<<","<<y<<","<<z<<")"<<std::endl;
}

void callback(const  sensor_msgs::PointCloud2& msg){
	//Point cloud Vector
	//pcl::PointCloud<PointType>::Ptr vizClouds(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
	//std::printf(" Visualser\n");
	pcl::fromROSMsg(msg,*trcloud2);
	//std::cout<<trcloud2->points[0]<<std::endl;
	vizClouds->width=trcloud2->width;
   	vizClouds->height=trcloud2->height;
   	vizClouds->is_dense=true;
	vizClouds->points.resize(trcloud2->width*trcloud2->height*total);
	for(int r=0;r<vizClouds->width*vizClouds->height;++r){	 
		vizClouds->points.push_back(trcloud2->points[r]);
	}
        
	//create filtering object
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud (vizClouds);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (-2000,2000); //2600
	//pass.setFilterLimitsNegative(true);
	pass.filter (*filtered_cloud);

	// Update Point Cloud (Viewewr)
	handler->setInputCloud(filtered_cloud);
	if( !viewer->updatePointCloud( filtered_cloud, *handler, "trcloud" ) ){
		viewer->addPointCloud( filtered_cloud, *handler, "trcloud" );
        }
	vizClouds->points.clear();
        
	// Update Viewer
        viewer->spinOnce();

	ros::spinOnce();
}

int main(int argc, char** argv){
	printf("pcd visualizer\n");
	initVis();
	ros::init(argc, argv, "sub_aln_visualizer");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2 point2;
	ros::Subscriber sub = nh.subscribe("align_cloud",1,callback);
	ros::spin();
	//visualiser();

}
