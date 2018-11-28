#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointXYZI PointType;
//pcl::PointCloud<PointType> trcloud;
pcl::PointCloud<PointType>::Ptr trcloud2(new pcl::PointCloud<PointType>);

//use pcl viewer
// PCL Visualizer
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "pcl Viewer" ) );
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
    	viewer->setCameraPosition( 20.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0 );

        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType> > color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;
}

//paramater for camera position in viewer
static float cam_yaw=0,cam_pitch=0,cam_roll=0;
static float cam_x=0,cam_y=0,cam_z=0;
static float cam_zoom=20;
void js_callback(const  std_msgs::Float32MultiArray& cmd_ctrl){
	//printf("callback\n");
	std::cout<<"[0]"<<(int)cmd_ctrl.data[0]<<" [1]"<<(int)cmd_ctrl.data[1]<<" [2]"<<(int)cmd_ctrl.data[2]
		 <<" [3]"<<(int)cmd_ctrl.data[3]<<" [4]"<<(int)cmd_ctrl.data[4]<<" [5]"<<(int)cmd_ctrl.data[5]<<std::endl;
	cam_yaw   += (float)cmd_ctrl.data[6];
	cam_pitch += (float)cmd_ctrl.data[7];
	cam_roll  += (float)cmd_ctrl.data[7];
	cam_x     += (float)cmd_ctrl.data[8];
	cam_y     += (float)cmd_ctrl.data[9];
	cam_z     += (float)cmd_ctrl.data[9];
	cam_zoom  += (float)cmd_ctrl.data[10];
	// Setting camera paramater
	viewer->setCameraPosition( cam_zoom+cam_x, cam_zoom+cam_y, cam_zoom+cam_z, cam_yaw, cam_pitch, cam_roll, cam_yaw, cam_pitch, cam_roll, 0);
	viewer->spinOnce();
}

void callback(const  sensor_msgs::PointCloud2& msg){
	//Point cloud Vector
	//pcl::PointCloud<PointType>::Ptr vizClouds(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr filtered_cloud2(new pcl::PointCloud<PointType>);
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
	if(total>10){
		//create filtering object
		/*pcl::PassThrough<pcl::PointXYZI> pass;
 		pass.setInputCloud (vizClouds);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-2000,20000); //2600
		//pass.setFilterLimitsNegative(true);
		pass.filter (*filtered_cloud);

		// Update Point Cloud (Viewewr)
		handler->setInputCloud(filtered_cloud);
		if( !viewer->updatePointCloud( filtered_cloud, *handler, "trcloud" ) ){
			viewer->addPointCloud( filtered_cloud, *handler, "trcloud" );
			}*/

		//create z filtering object
		pcl::PassThrough<pcl::PointXYZI> pass_z;
		pass_z.setInputCloud (vizClouds);
		pass_z.setFilterFieldName ("z");
		pass_z.setFilterLimits (-20000,20000); //2600
		//pass.setFilterLimitsNegative(true);
		pass_z.filter (*filtered_cloud);

		//create y filtering object
		pcl::PassThrough<pcl::PointXYZI> pass_y;
		pass_y.setInputCloud (filtered_cloud);
		pass_y.setFilterFieldName ("y");
		pass_y.setFilterLimits (-500,500); //2600
		//pass.setFilterLimitsNegative(true);
		pass_y.filter (*filtered_cloud2);

		// Update Point Cloud (Viewewr)
		handler->setInputCloud(filtered_cloud);
		if( !viewer->updatePointCloud( filtered_cloud, *handler, "trcloud" ) ){
			viewer->addPointCloud( filtered_cloud, *handler, "trcloud" );
		}

		vizClouds->points.clear();
		total=0;
	}
	total++;
	// Setting camera paramater
	//viewer->setCameraPosition( cam_yaw, cam_pitch, cam_roll, cam_x, cam_y, cam_z, cam_zoom );
	// Update Viewer
        viewer->spinOnce();

	ros::spinOnce();
}

int main(int argc, char** argv){
	printf("pcd visualizer\n");
	initVis();
	ros::init(argc, argv, "sub_pcl_visualizer");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2 point2;
	ros::Subscriber sub = nh.subscribe("point2",1,callback);
	ros::Subscriber sub1 = nh.subscribe("cmd_ctrl",1,js_callback);
	ros::spin();
	//visualiser();

}
