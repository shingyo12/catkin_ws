#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl/ros/conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>

sensor_msgs::PointCloud2 PointCloud;

void callback(const  sensor_msgs::PointCloud2& pcd_msg){
	printf("Cloud: width = %d,height = %d\n",pcd_msg.width,pcd_msg.height);
}

void messageCallBack(const std_msgs::Float64& cy_msg){
	ROS_INFO_STREAM(cy_msg.data);
}

int main(int argc, char** argv){
	printf("record point_cloud_data & swing_cycle\n");
	ros::init(argc, argv, "pcd_record");
	ros::NodeHandle nh;
	ros::Subscriber sub1 = nh.subscribe("row_points",1,callback);
	ros::Subscriber sub2 = nh.subscribe("cycle",1,&messageCallBack);
	ros::spin();
}
