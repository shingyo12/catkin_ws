#include <iostream>
#include <string>
#include <vector>
#include <fstream> 
#include <math.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#define NoM 5 //Number of Measurement (rotational number of laser element in LIDAR)

int wait_cb=0,rot_cnt=0;

sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointXYZI PointType;
//pcl::PointCloud<PointType> trcloud;
pcl::PointCloud<PointType>::Ptr trcloud2(new pcl::PointCloud<PointType>);
//Point cloud Vector
pcl::PointCloud<PointType>::Ptr pushClouds(new pcl::PointCloud<PointType>);

geometry_msgs::Pose sensor;

void gps_transformation(pcl::PointCloud<PointType>::Ptr pushClouds){
	ROS_INFO_STREAM("point cloud move to gps position");
	for(int r=0;r<pushClouds->width*pushClouds->height;++r){	 
		pushClouds->points[r].x=pushClouds->points[r].x+sensor.position.x;
		pushClouds->points[r].y=pushClouds->points[r].y+sensor.position.y;
	}
	ros::NodeHandle nh1;
	ros::Publisher pub1  = nh1.advertise<sensor_msgs::PointCloud2>("gps_points",10);
	pub1.publish(pushClouds);
}

void subscribeCallback(const geometry_msgs::Pose& gps){
	//ROS_INFO_STREAM("subscribe callback");
	ROS_INFO("x:%lf y:%lf",gps.position.x,gps.position.y);
	sensor.position.x=gps.position.x;
	sensor.position.y=gps.position.y;
	wait_cb=0;
}

void pcd_callback(const  sensor_msgs::PointCloud2& msg){
	static int total=1;
	//std::printf(" Visualser\n");
	pcl::fromROSMsg(msg,*trcloud2);
	pushClouds->width+=trcloud2->width;
   	pushClouds->height=trcloud2->height;
   	pushClouds->is_dense=true;
	//pushClouds->points.resize(trcloud2->width*trcloud2->height*rot_cnt);
	for(int r=0;r<trcloud2->width*trcloud2->height;++r){	 
		pushClouds->points.push_back(trcloud2->points[r]);
	}
	if(rot_cnt>NoM){
		//gps_transformation(pushClouds);
		ROS_INFO_STREAM("point cloud move to gps position");
		for(int r=0;r<pushClouds->width*pushClouds->height;++r){	 
			pushClouds->points[r].x=pushClouds->points[r].x+sensor.position.x;
			pushClouds->points[r].y=pushClouds->points[r].y+sensor.position.y;
		}
		ros::NodeHandle nh1;
		ros::Publisher pub1  = nh1.advertise<sensor_msgs::PointCloud2>("gps_points",10);
		pub1.publish(pushClouds);
		
		pushClouds->points.clear();
		pushClouds->width=0;
		total=1;
	}
	rot_cnt++;
}

int main(int argc, char **argv){
	int mes_cnt=0;
	std_msgs::Float64 flt_msg;
	char ch;
	int k=0;
	
	ros::init(argc, argv, "rec_pcd_gps");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Float64>("start_gps",1);
	ros::Subscriber sub1 = nh.subscribe("gps_data",1,&subscribeCallback); 
	ros::ServiceClient testClient = nh.serviceClient<std_srvs::Empty>("get_gps");
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;

	sensor_msgs::PointCloud2 point2;
	ros::Subscriber sub2 = nh.subscribe("point2",10,pcd_callback);

	ros::Rate rate(1);
	while(ros::ok()){
		if(rot_cnt==0){
			//ROS_INFO_STREAM("send start message");
			flt_msg.data=mes_cnt;
			pub.publish(flt_msg);
			if(k==0){
				ROS_INFO_STREAM("please wait .....");
				k=1;
			}else{
				ROS_INFO_STREAM("now measuring .....");
			}
		}
		//rot_cnt++;
		if(rot_cnt>NoM){
			//ROS_INFO_STREAM("send request");
			bool result = testClient.call(req, resp);
			/*if(result){
				ROS_INFO_STREAM("Recieve response");
       			}else{
				ROS_INFO_STREAM("Error");
				}*/
			rot_cnt=0;
			wait_cb=1;
			while(1){
				ros::spinOnce();
				if(wait_cb==0)break;
			}
			
			std::cout<<"start measurement -> input s and enter"<<std::endl;
		       	while(1){
				std::cin>>ch;
			       	if(ch=='s')break;
		       	}
			mes_cnt++;
		}
		//rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
