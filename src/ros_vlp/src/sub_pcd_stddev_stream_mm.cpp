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

//file stream
fstream fs;
fstream fs1;

#define MAX_Dp 10000
#define MAX_Dm 0
#define MAX_H 500

struct stddev_dots{
	double x;
	double z;
	double err;

	stddev_dots(double x,double z,double err) : x(x),z(z),err(err) {}
};

struct grid_cell{
	int    pt_num=0;

	double sum_x=0;
	double sum_y=0;
	double sum_z=0;
	double sum_intensity=0;
	double sum_err=0;

	double sum_xz=0;
	double sum_xx=0;

	double ave_x=0;
	double ave_y=0;
	double ave_z=0;
	double ave_intensity=0;
	double ave_err=0;

	double sq_sum=0;
	double std_dev=0;
};

sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointXYZI PointType;
//pcl::PointCloud<PointType> trcloud;
pcl::PointCloud<PointType>::Ptr trcloud2(new pcl::PointCloud<PointType>);

//Point cloud Vector
pcl::PointCloud<PointType>::Ptr vizClouds(new pcl::PointCloud<PointType>);
int total=1,j=0,cnt=0;
std::vector<grid_cell> grid;
double a0,a1,a00=0,a01=0,a02=0,a11=0,a12=0;

void standard_deviation(pcl::PointCloud<PointType>::Ptr vizClouds,int total){
	std::cout<<"standard_deviation  "<<vizClouds->width<<"  "<<vizClouds->height<<std::endl;
	//fs<<"standard_deviation  "<<vizClouds->width<<"  "<<vizClouds->height<<std::endl;
	std::vector<stddev_dots> object_points;
	//std::vector<grid_cell> grid;
	grid.resize(MAX_Dp/100);
	//static double a0,a1,a00=0,a01=0,a02=0,a11=0,a12=0;
	double d_err;
	static int grid_num;
	for(int i=0;i<vizClouds->width*vizClouds->height;i++){
		//approximate straight line : z=a0+a1*x
		if(-50<vizClouds->points[i].y && 50>vizClouds->points[i].y 
				&& vizClouds->points[i].y != 0 && vizClouds->points[i].z != 0 
					&& MAX_H>vizClouds->points[i].z && MAX_Dp>vizClouds->points[i].x && MAX_Dm<vizClouds->points[i].x){
			a00 += 1.0;
			a01 += vizClouds->points[i].x;
			a02 += vizClouds->points[i].z;
			a11 += vizClouds->points[i].x * vizClouds->points[i].x;
			a12 += vizClouds->points[i].z * vizClouds->points[i].z;
		}
		//a0=(a02*a11-a01*a02)/(a00*a11-a01*a01);
		//a1=(a00*a12-a01*a02)/(a00*a11-a01*a01);
	}
	for(int i=0;i<vizClouds->width*vizClouds->height;i++){
		//pick up : object points
		if(-50<vizClouds->points[i].y && 50>vizClouds->points[i].y 
				&& vizClouds->points[i].y != 0 && vizClouds->points[i].z != 0 
					&& MAX_H>vizClouds->points[i].z && MAX_Dp>vizClouds->points[i].x && MAX_Dm<vizClouds->points[i].x){

			d_err = vizClouds->points[i].z-(a0+a1*vizClouds->points[i].y);

			object_points.emplace_back(vizClouds->points[i].y ,vizClouds->points[i].z ,d_err);

			std::cout<<vizClouds->points[i].x<<"  "<<vizClouds->points[i].y<<"  "<<vizClouds->points[i].z<<std::endl;

			fs<<vizClouds->points[i].x<<"  "<<vizClouds->points[i].y<<"  "<<vizClouds->points[i].z<<std::endl;

			for(int j=0;j<MAX_Dp/100;j++){
				if((j+1)*100>vizClouds->points[i].x && j*100<vizClouds->points[i].x){
					grid[j].pt_num++;
					grid[j].sum_x += vizClouds->points[i].x;
					grid[j].sum_y += vizClouds->points[i].y;
					grid[j].sum_z += vizClouds->points[i].z;
					grid[j].sum_intensity += vizClouds->points[i].intensity;
					grid[j].sum_err += fabs(vizClouds->points[i].z-(a0+a1*vizClouds->points[i].y));
					grid[j].sq_sum += vizClouds->points[i].z * vizClouds->points[i].z;
					grid[j].sum_xx += vizClouds->points[i].x * vizClouds->points[i].x;
					grid[j].sum_xz += vizClouds->points[i].x * vizClouds->points[i].z;
				}
				//fs1<<j<<"  "<<grid[j].pt_num<<"  "<<grid[j].ave_x<<"  "<<grid[j].ave_y<<"  "<<grid[j].ave_z<<"  "<<
						//grid[j].ave_intensity<<"  "<<grid[j].ave_err<<"  "<<grid[j].std_dev<<std::endl;
			}
		}
	}
	if(cnt>30){
		std::printf(" ok\n");
		a0=(a02*a11-a01*a02)/(a00*a11-a01*a01);
		a1=(a00*a12-a01*a02)/(a00*a11-a01*a01);
		fs1<<"a0(intercept):"<<a0<<"  a1(gradent):"<<a1<<std::endl;
		for(int k=0;k<MAX_Dp/100;k++){
				grid[k].ave_x = grid[k].sum_x / grid[k].pt_num;
				grid[k].ave_y = grid[k].sum_y / grid[k].pt_num;
				grid[k].ave_z = grid[k].sum_z / grid[k].pt_num;
				//grid[k].ave_intensity = grid[k].sum_intensity / grid[k].pt_num;
				//grid[k].ave_err = grid[k].sum_err / grid[k].pt_num;
				//grid[k].std_dev = sqrt(grid[k].sq_sum / grid[k].pt_num - grid[k].ave_z*grid[k].ave_z);
			fs1<<k<<"  "<<grid[k].pt_num<<"  "<<grid[k].ave_x<<"  "<<grid[k].ave_y<<"  "<<grid[k].ave_z<<"  "<<
						grid[k].sum_x<<"  "<<grid[k].sum_z<<"  "<<grid[k].sum_xz<<"  "<<grid[k].sum_xx<<std::endl;
		}
		cnt=0;
		fs.close();
		fs1.close();
		while(ros::ok()){
			sleep(1);
		}
	}
}

void callback(const  sensor_msgs::PointCloud2& msg){
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

		vizClouds->points.clear();
		total=1;
		cnt++;
	}
	total++;
	if(!ros::ok()){
		std::printf(" ok\n");
		for(int k=0;k<MAX_Dp/100;k++){
			fs1<<k<<"  "<<grid[k].pt_num<<"  "<<grid[k].ave_x<<"  "<<grid[k].ave_y<<"  "<<grid[k].ave_z<<"  "<<
						grid[k].ave_intensity<<"  "<<grid[k].ave_err<<"  "<<grid[k].std_dev<<std::endl;
		}
		fs.close();
		fs1.close();
	}
	ros::spin();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sub_pcl_stddev_stream");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2 point2;
	ros::Subscriber sub = nh.subscribe("point2",10,callback);

	//file stream setup
	fs.open("object_points.txt", ios::out);
	if(! fs.is_open()){
		printf("failed open file : object_points.txt");
	}
	fs1.open("grids.txt", ios::out);
	if(! fs.is_open()){
		printf("failed open file : grids.txt");
	}

	ros::spin();
}
