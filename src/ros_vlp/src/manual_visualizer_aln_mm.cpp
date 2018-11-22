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

typedef pcl::PointXYZI PointType;
pcl::PointCloud<PointType> filtered_cloud_g;
std::string fname;
std::string fstyle;

void dispersion(float x1,float y1,float x2,float y2){
	static int cnt=0;
	int points_num=0;
	float xmax,xmin,ymax,ymin;
	std::string out_name;
	if(x1>x2){
	        xmax=x1;
		xmin=x2;
	}else{
		xmax=x2;
		xmin=x1;
	}
	if(y1>y2){
		ymax=y1;
		ymin=y2;
	}else{
		ymax=y2;
		ymin=y1;
	}
	pcl::PointCloud<PointType> pickup;
	std::cout<<"xmax:"<<xmax<<" ymax:"<<ymax<<" xmin:"<<xmin<<" ymin:"<<ymin<<std::endl;
	for(int i=0;i<filtered_cloud_g.points.size();i++){
		if(filtered_cloud_g.points[i].x>xmin && filtered_cloud_g.points[i].x<xmax &&
		              filtered_cloud_g.points[i].y>ymin && filtered_cloud_g.points[i].y<ymax ){
			pickup.points.push_back(filtered_cloud_g.points[i]);
			printf("%d\n",i);
			points_num++;
		}
	}
	pickup.width=points_num;
	pickup.height=1;
	pickup.is_dense=true;
	out_name="pickup_"+fname;
	if(fstyle=="b"){
		pcl::io::savePCDFileBinary(out_name,pickup);
	}else{
		pcl::io::savePCDFileASCII(out_name,pickup);
	}
	std::cout<<"saved : "<<out_name<<std::endl;
	cnt++;
}

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void){
	static int cb_cnt=0;
	static float x1,y1,x2,y2,xmax,xmin,ymax,ymin;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer=*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	float x,y,z;
	if(event.getPointIndex() == -1){
		return;
	}
        event.getPoint(x,y,z);
	//std::cout<<"point"<<cb_cnt<<" ("<<x<<","<<y<<","<<z<<")"<<std::endl;
	std::cout<<x<<" "<<y<<" "<<z<<std::endl;
	if(cb_cnt==0){
		x1=x;
		y1=y;
		cb_cnt=1;
	}else{
		x2=x;
		y2=y;
		cb_cnt=0;
		dispersion(x1,y1,x2,y2);
	}
		//cb_cnt++;
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
	//std::string fname;
	if(argc==3){
		std::stringstream ss;
		ss<<argv[1];
		fname=ss.str();
		std::cout<<fname<<" is opened"<<std::endl;
		fstyle=argv[2];
	}else{
		std::cout<<"Please input PCD file name"<<std::endl;
		return 0;
	}
	//initVis();
	sensor_msgs::PointCloud2 PointCloud;
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

        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType> > color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "z" ) );

        handler = color_handler;

	viewer->registerPointPickingCallback(pp_callback, (void*)&viewer);
	viewer->registerAreaPickingCallback(pp_callback2, (void*)&viewer);
	
	ros::init(argc, argv, "manual_visualizer");
	ros::NodeHandle nh;

	//visualizer();
	//Point cloud Vector
	pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
	//pcl::PointCloud<PointType>::Ptr filtered_cloud2(new pcl::PointCloud<PointType>);
	//std::printf(" Visualser\n");
 
	pcl::io::loadPCDFile(fname,*vizClouds);
	
        //create z filtering object
        pcl::PassThrough<pcl::PointXYZI> pass_z;
        pass_z.setInputCloud (vizClouds);
        pass_z.setFilterFieldName ("y");
        pass_z.setFilterLimits (-2000,200000); //2600
        //pass.setFilterLimitsNegative(true);
        pass_z.filter (*filtered_cloud);

	//create y filtering object
        /*pcl::PassThrough<pcl::PointXYZI> pass_y;
        pass_y.setInputCloud (filtered_cloud);
        pass_y.setFilterFieldName ("y");
        pass_y.setFilterLimits (0,1500); //2600
        //pass.setFilterLimitsNegative(true);
        pass_y.filter (*filtered_cloud2);*/

	filtered_cloud_g=*filtered_cloud;

	std::cout<<"Shift+click two points"<<std::endl;

        // Update Point Cloud (Viewewr)
	handler->setInputCloud(vizClouds);
        if( !viewer->updatePointCloud( vizClouds, *handler, "trcloud" ) ){
		viewer->addPointCloud( vizClouds, *handler, "trcloud" );
	}

	// Update Viewer
        viewer->spin();
	
	//ros::spin();
}
