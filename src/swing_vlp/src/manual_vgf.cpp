//#include "stdafx.h"
#include <iostream>
#include <string>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
//#include "pcl/cuda/filters/include/pcl/cuda/filters/voxel_grid.h"

#define VOX_SIZE 0.3

pcl::PointCloud<pcl::PointXYZI> voxel_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

	//pcl_cuda::VoxelGrid<pcl::PointXYZI> sor;
	pcl::VoxelGrid<pcl::PointXYZI> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (VOX_SIZE, VOX_SIZE, VOX_SIZE);
	sor.filter (*cloud_filtered);

	std::cout<<"filtered point cloud size : "<<cloud_filtered->points.size()<<std::endl;

	return *cloud_filtered;
}

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

	ros::init(argc, argv, "manual_vgf");
	ros::NodeHandle nh;
	
	std::stringstream ss_in,ss_out;
	ss_in<<argv[1];
	
	pcl::io::loadPCDFile(ss_in.str(), *cloud);
	std::cout<<"load point cloud size : "<<cloud->points.size()<<std::endl;

	std::string out_name;
	std::string name=ss_in.str();
	name.erase(name.length()-4);
	ss_out << name << "_voxel.pcd";

	pcl::io::savePCDFileBinary(ss_out.str(), voxel_grid(cloud));
	std::cout<<"saved : "<<ss_out.str()<<std::endl;

	ros::spinOnce();

	return (0);  
}
