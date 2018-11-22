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

void callback(const  sensor_msgs::PointCloud2& msg){
	printf("Cloud: width = %d,height = %d\n",msg.width,msg.height);
}

int main(int argc, char** argv){
	printf("test for pointcloud2\n");
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("point2",1,callback);
	ros::spin();
}
