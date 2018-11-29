#include <iostream>
#include <string>
#include <vector>
#include <fstream> 
#include <math.h>
#include <unistd.h>
#include <time.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#define NoM 10 //Number of Measurement (rotational number of laser element in LIDAR)

sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointXYZI PointType;
//pcl::PointCloud<PointType> trcloud;

struct position {
	double x;
	double y;
};

position sensor_pos;
int gps_flg=0;
int start_flg=0;

pcl::PointCloud<PointType>::Ptr trcloud2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr push_cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr moved_cloud(new pcl::PointCloud<PointType>);

void js_callback(const  std_msgs::Float32MultiArray& cmd_ctrl){
	//printf("callback\n");
	std::cout<<"[0]"<<(int)cmd_ctrl.data[0]<<" [1]"<<(int)cmd_ctrl.data[1]<<" [2]"<<(int)cmd_ctrl.data[2]
		 <<" [3]"<<(int)cmd_ctrl.data[3]<<" [4]"<<(int)cmd_ctrl.data[4]<<" [5]"<<(int)cmd_ctrl.data[5]<<std::endl;
	if((int)cmd_ctrl.data[0] == 1){
		start_flg = 1;
	}
	if((int)cmd_ctrl.data[1] == 1){
		start_flg = 0;
	}
}

void gps_callback(const  geometry_msgs::Pose& mgs){
	sensor_pos.x=mgs.position.y;
	sensor_pos.y=mgs.position.x;
	gps_flg=1;
}

void pcd_callback(const  sensor_msgs::PointCloud2& msg){
	static int total=1;
	double man_x,man_y;
	int ix,iy,num;
	//std::printf(" Visualser\n");
	//push_cloud->points.resize(trcloud2->width*trcloud2->height*total);
	if(start_flg>0){
		printf("start flg %d\n",start_flg);

		pcl::fromROSMsg(msg,*trcloud2);

		push_cloud->width+=trcloud2->width;
		push_cloud->height=trcloud2->height;
		push_cloud->is_dense=true;
		for(int r=0;r<trcloud2->width*trcloud2->height;++r){	 
			push_cloud->points.push_back(trcloud2->points[r]);
		}
		if(total>NoM){
			printf("total %d\n",total);
			//man_x=sensor_pos.x;
			//man_y=sensor_pos.y;
			printf("sensor pos\n");
			for(int r=0;r<push_cloud->width*push_cloud->height;++r){	 
				push_cloud->points[r].x=push_cloud->points[r].x+sensor_pos.x;
				push_cloud->points[r].y=push_cloud->points[r].y+sensor_pos.y;
			}
		
			//ix=(int)man_x;
			//iy=(int)man_y;

			//push_cloud->sensor_origin_.x()=sensor_pos.x;
			//push_cloud->sensor_origin_.y()=sensor_pos.y;
		
			char date[64];
			time_t t = time(NULL);
			printf("getting date and time\n");
			strftime(date, sizeof(date), "%Y%m%d%H%M%s", localtime(&t));
			std::stringstream ss;
			printf("string stream\n");
			ss<<"~/pcd_data/"<<date<<".pcd";
			std::string fname=ss.str();
			std::cout<<"File name : "<<fname<<std::endl;
			pcl::io::savePCDFileBinary(fname,*push_cloud);
			std::cout<<"File saved"<<std::endl;
			push_cloud->points.clear();
			//exit(0);
			total=1;
			start_flg=0;
			push_cloud->width=0;
			push_cloud->height=0;
		}
		total++;
	}
	ros::spinOnce();
}


int main(int argc, char **argv){
	int mes_cnt=0;
	std_msgs::Float64 flt_msg;
	char ch;
	int k=0;
	
	ros::init(argc, argv, "pcd_recoard");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2 point2;
	ros::Subscriber sub0 = nh.subscribe("cmd_ctrl",1,js_callback);
	ros::Subscriber sub1 = nh.subscribe("gps",1,gps_callback);
	ros::Subscriber sub2 = nh.subscribe("point2",1,pcd_callback);
	/*while(ros::ok()){
		ros::spinOnce();
		//if(gps_flg==1)break;
		}*/

	ros::spin();
	
	return 0;
}
