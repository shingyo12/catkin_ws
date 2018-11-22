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
	std::string in_fname,out_fname;
	if(argc==3){
		std::stringstream ss_in,ss_out;
		ss_in<<argv[1];
		in_fname=ss_in.str();
		std::cout<<in_fname<<" is opened"<<std::endl;

		ss_out<<argv[2];
		out_fname=ss_out.str();
		std::cout<<"output to "<<out_fname<<std::endl;
	}else{
		std::cout<<"Please input PCD file name"<<std::endl;
		return 0;
	}
	//initVis();
	sensor_msgs::PointCloud2 PointCloud;
	typedef pcl::PointXYZI PointType;
	//pcl::PointCloud<PointType> trcloud;
	pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr middle_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
	//pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
	
	ros::init(argc, argv, "manual_transform");
	ros::NodeHandle nh;

	//std::printf(" Visualser\n");
 
	pcl::io::loadPCDFile(in_fname,*input_cloud);

	double x,y,z,yaw,pitch,roll;
	std::cout<<"input amount of movement"<<std::endl;
	std::cout<<"x[mm] : ";
	cin>>x;
	std::cout<<"y[mm] : ";
	cin>>y;
	std::cout<<"z[mm] : ";
	cin>>z;
        std::cout<<"yaw[deg] : ";
	cin>>yaw;
	std::cout<<"pitch[deg] : ";
	cin>>pitch;
	std::cout<<"roll[deg] : ";
	cin>>roll;

	double r=yaw*M_PI/180;
	double p=pitch*M_PI/180;
	double ya=roll*M_PI/180;

	double cosY=cos(ya);
	double cosP=cos(p);
	double cosR=cos(r);
	double sinY=sin(ya);
	double sinP=sin(p);
	double sinR=sin(r);

	middle_cloud->width=input_cloud->width;
	middle_cloud->height=input_cloud->height;
	middle_cloud->is_dense=true;
	middle_cloud->points.resize(middle_cloud->width*middle_cloud->height);
	for(int i=0;i<=middle_cloud->points.size();i++){
		middle_cloud->points[i].x=cosP*cosR*input_cloud->points[i].x
						+ (sinY*sinP*cosR-cosY*sinR)*input_cloud->points[i].y
							+ (sinY*sinR+cosY*sinP*cosR)*input_cloud->points[i].z; 
		middle_cloud->points[i].y=cosP*sinR*input_cloud->points[i].x
						+ (sinY*sinP*sinR+cosY*cosR)*input_cloud->points[i].y
							+ (-sinY*cosR+cosY*sinP*sinR)*input_cloud->points[i].z;
		middle_cloud->points[i].z=-sinP*input_cloud->points[i].x
						+sinY*cosP*input_cloud->points[i].y
							+cosY*cosP*input_cloud->points[i].z;
	}

	output_cloud->width=input_cloud->width;
	output_cloud->height=input_cloud->height;
	output_cloud->is_dense=true;
	output_cloud->points.resize(output_cloud->width*output_cloud->height);
	for(int i=0;i<=middle_cloud->points.size();i++){
		output_cloud->points[i].x=middle_cloud->points[i].x + x; 
		output_cloud->points[i].y=middle_cloud->points[i].y + y;
		output_cloud->points[i].z=middle_cloud->points[i].z + z;
	}

	pcl::io::savePCDFileBinary(out_fname,*output_cloud);
	std::cout<<"Saved"<<std::endl;
	//ros::spin();
}
