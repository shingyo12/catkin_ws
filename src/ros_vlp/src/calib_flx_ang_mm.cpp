#include <iostream>
#include <string>
#include <vector>
#include <fstream>

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

#include "VLP16.h"
#include "calib.h"

#define MAX_D 10000
#define MAX_H 500

double tmp_flxang;

struct stddev_dots{
	double x;
	double z;
	double err;

	stddev_dots(double x,double z,double err) : x(x),z(z),err(err) {}
};

struct grid_cell{
	int    pt_num=0;

	double sum_x = 0;
	double sum_y = 0;
	double sum_z = 0;
	double sum_intensity = 0;
	double sum_err = 0;

	double ave_x = 0;
	double ave_y = 0;
	double ave_z = 0;
	double ave_intensity = 0;
	double ave_err = 0;

	double sq_sum = 0;
	double std_dev = 0;
};

sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointXYZI PointType;
//pcl::PointCloud<PointType> trcloud;
pcl::PointCloud<PointType>::Ptr trcloud2(new pcl::PointCloud<PointType>);

//Point cloud Vector
pcl::PointCloud<PointType>::Ptr vizClouds(new pcl::PointCloud<PointType>);
int total=1,j=0;
double p_ave_err=0;

void standard_deviation(pcl::PointCloud<PointType>::Ptr vizClouds,int total){

	//ros::NodeHandle pnh;
	//ros::Publisher pub = pnh.advertise<std_msgs::Float64>("calib_flxang",1);

	std::cout<<"standard_deviation  "<<vizClouds->width<<"  "<<vizClouds->height<<std::endl;
	std::vector<stddev_dots> object_points;
	std::vector<grid_cell> grid;
	grid.resize(MAX_D/100);
	double a0,a1,a00=0,a01=0,a02=0,a11=0,a12=0;
	double d_err,sum_err=0,ave_err;
	int pt_num=0;
	//double calib_flxang;
	static int cnt=0;
	static double better_flxang = START_ANG;
	//std_msgs::Float64 calib_flxang;
	for(int i=0;i<vizClouds->width*vizClouds->height;i++){
		//approximate straight line : z=a0+a1*x
		if(-50<vizClouds->points[i].y && 50>vizClouds->points[i].y 
				&& vizClouds->points[i].y != 0 && vizClouds->points[i].z != 0 
					&& MAX_H>vizClouds->points[i].z && MAX_D>vizClouds->points[i].x && 0<vizClouds->points[i].x){
			a00 += 1.0;
			a01 += vizClouds->points[i].y;
			a02 += vizClouds->points[i].z;
			a11 += vizClouds->points[i].y * vizClouds->points[i].y;
			a12 += vizClouds->points[i].z * vizClouds->points[i].z;
		}
	}
	a0=(a02*a11-a01*a02)/(a00*a11-a01*a01);
	a1=(a00*a12-a01*a02)/(a00*a11-a01*a01);
	for(int i=0;i<vizClouds->width*vizClouds->height;i++){
		//pick up : object points
		if(-50<vizClouds->points[i].y && 50>vizClouds->points[i].y 
				&& vizClouds->points[i].y != 0 && vizClouds->points[i].z != 0 
					&& MAX_H>vizClouds->points[i].z && MAX_D>vizClouds->points[i].x && 0<vizClouds->points[i].x){

			d_err = vizClouds->points[i].z-(a0+a1*vizClouds->points[i].x);

			object_points.emplace_back(vizClouds->points[i].y ,vizClouds->points[i].z ,d_err);

			std::cout<<vizClouds->points[i].x<<"  "<<vizClouds->points[i].y<<"  "<<vizClouds->points[i].z
												<<"  "<<d_err<<"  "<<fabs(d_err)<<std::endl;
			pt_num++;
			sum_err += abs(d_err);
		}
	}
	if(pt_num != 0){
		ave_err=sum_err/pt_num;
		tmp_flxang = (START_ANG + STRIDE_ANG*cnt) - Flexion_Angle;
		if(p_ave_err > ave_err) better_flxang = Flexion_Angle + tmp_flxang;
		std::cout<<cnt<<"  "<<"average_Z_error:"<<ave_err<<" "<<p_ave_err<<"  calib_ang:"<<Flexion_Angle + tmp_flxang<<"  better_ang:"<<better_flxang<<std::endl;
		//pub.publish(calib_flxang);
		p_ave_err = ave_err;
	}
	cnt++;
	if(calib_times<cnt){
		cnt=0;
	}
}
void callback(const  sensor_msgs::PointCloud2& msg){
	ros::NodeHandle nh1;
	ros::Publisher pub = nh1.advertise<std_msgs::Float64>("calib_flxang",1);
	std_msgs::Float64 calib_flxang;
	//std::printf(" Visualser\n");
	pcl::fromROSMsg(msg,*trcloud2);
	//std::cout<<trcloud2->points[0]<<std::endl;
	vizClouds->width=trcloud2->width*total;
   	vizClouds->height=trcloud2->height;
   	vizClouds->is_dense=true;
	vizClouds->points.resize(trcloud2->width*trcloud2->height*total);
	for(int r=0;r<vizClouds->width*vizClouds->height;++r){	 
		vizClouds->points.push_back(trcloud2->points[r]);
	}
	if(total>11){
		standard_deviation(vizClouds,total);
		calib_flxang.data = tmp_flxang;
		pub.publish(calib_flxang);
		vizClouds->points.clear();
		total=1;
	}
	total++;
	ros::spin();
}

int main(int argc, char** argv){
	printf("pcd visualizer\n");
	ros::init(argc, argv, "calib_flx_ang");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2 point2;
	ros::Subscriber sub = nh.subscribe("point2",10,callback);
	//ros::Publisher pub = nh.advertise<std_msgs::Float64>("calib_flxang",1);
	//std_msgs::Float64 calib_flxang;
	//calib_flxang.data = tmp_flxang;
	//pub.publish(calib_flxang);
	
	ros::spin();
}
