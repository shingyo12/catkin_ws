#include <iostream>
#include <string>
#include <vector>
#include <fstream> 

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#define BIN_FILE_NAME "combined_cloud.pcd"

sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointXYZI PointType;

pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr tgt_cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType> align_cloud;
pcl::PointCloud<PointType>::Ptr cmb_cloud(new pcl::PointCloud<PointType>);

void pcd_callback(const  sensor_msgs::PointCloud2& msg){	
       	static int icp_cnt=0;
	std::printf(" pcd_callback\n");
	pcl::fromROSMsg(msg,*src_cloud);
	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

	ros::NodeHandle nh1;
	ros::Publisher pub = nh1.advertise<sensor_msgs::PointCloud2>("align_cloud",1);
	sensor_msgs::PointCloud2 align_point2;
	
	if(icp_cnt==0){
		*tgt_cloud=*src_cloud;

		cmb_cloud->width=tgt_cloud->width;
		cmb_cloud->height=tgt_cloud->height;
		cmb_cloud->is_dense=true;
		for(int r=0;r<tgt_cloud->points.size();++r){	 
			cmb_cloud->points.push_back(tgt_cloud->points[r]);
		}
	}
	else{
		icp.setInputSource(src_cloud);
		icp.setInputTarget(tgt_cloud);
		icp.align(align_cloud);
		*tgt_cloud=*cmb_cloud;
		
		cmb_cloud->width+=align_cloud.width;
		for(int r=0;r<align_cloud.points.size();++r){	 
			cmb_cloud->points.push_back(align_cloud.points[r]);
		}
	}
	pcl::io::savePCDFileBinary(BIN_FILE_NAME,*cmb_cloud);
	pcl::toROSMsg(*cmb_cloud,align_point2);
	pub.publish(align_point2);
	icp_cnt++;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "sub_icp");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2 point2;
	ros::Subscriber sub2 = nh.subscribe("point2",10,pcd_callback);

	ros::spin();

	return 0;
}
