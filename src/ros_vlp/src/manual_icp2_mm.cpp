#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <time.h>

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

typedef pcl::PointXYZRGB PointType;

/*void icp_align(pcl::PointCloud<PointType>::Ptr src_cloud,pcl::PointCloud<PointType>::Ptr tgt_cloud)
{
	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	pcl::PointCloud<PointType> align_cloud;
	struct timeval t0,t1;
	double caltime;
	std::string icp_name;
	std::cout<<"What is the name of combined file name?"<<std::endl;
	cin>>icp_name;
	gettimeofday(&t0,NULL);
	icp.setInputSource(src_cloud);
	icp.setInputTarget(tgt_cloud);
	icp.align(align_cloud);
	gettimeofday(&t1,NULL);
	
	std::cout<<"trabsformation matrix -> "<<std::endl;
	std::cout << icp.getFinalTransformation() <<std::endl;

	caltime=(t1.tv_sec-t0.tv_sec)+1e-6*(t1.tv_usec-t0.tv_usec);
	std::cout<<"calculation time : "<<caltime<<std::endl;
	
	pcl::io::savePCDFileBinary(icp_name,align_cloud);
	std::cout<<"Saved"<<std::endl;
	}*/

void points_i2rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr i_cloud,int r,int g,int b){
	rgb_cloud->width=i_cloud->width;
   	rgb_cloud->height=i_cloud->height;
   	rgb_cloud->is_dense=true;
   	rgb_cloud->points.resize(rgb_cloud->width*rgb_cloud->height);
	for(int r=0;r<rgb_cloud->points.size();++r){	 
		rgb_cloud->points[r].x=i_cloud->points[r].x;
		rgb_cloud->points[r].y=i_cloud->points[r].y;
		rgb_cloud->points[r].z=i_cloud->points[r].z;
		rgb_cloud->points[r].r=r;
		rgb_cloud->points[r].g=g;
		rgb_cloud->points[r].b=b;
        }
}

//be careful for pointer
void combine_points(pcl::PointCloud<PointType>::Ptr cmb_cloud,
		                pcl::PointCloud<PointType>::Ptr tgt_cloud,
		                        pcl::PointCloud<PointType>& align_cloud){
	//cmb_cloud->width=tgt_cloud->width+align_cloud.width;
	//cmb_cloud->height=align_cloud->height;
   	cmb_cloud->is_dense=true;
	//cmb_cloud->points.resize(align_cloud.points.size()+tgt_cloud->points.size());
	for(int r=0;r<tgt_cloud->points.size();++r){	 
		cmb_cloud->points.push_back(tgt_cloud->points[r]);
		}
	for(int r=0;r<align_cloud.points.size();++r){	 
		cmb_cloud->points.push_back(align_cloud.points[r]);
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "manual_icp");
	ros::NodeHandle nh;

	pcl::PointCloud<pcl::PointXYZI>::Ptr input_src_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_tgt_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr tgt_cloud(new pcl::PointCloud<PointType>);
	std::string source_name;
	std::string target_name;
	std::cout<<"What do you use Source file?"<<std::endl;
	cin>>source_name;
	std::cout<<"What do you use Target file?"<<std::endl;
	cin>>target_name;

	pcl::io::loadPCDFile(source_name,*input_src_cloud);
	pcl::io::loadPCDFile(target_name,*input_tgt_cloud);

	//icp_align(src_cloud,tgt_cloud);
	std::cout << "source file size:" << input_src_cloud->points.size() << std::endl;
	std::cout << "target file size:" << input_tgt_cloud->points.size() << std::endl;

	points_i2rgb(src_cloud,input_src_cloud,255,0,0);
	points_i2rgb(tgt_cloud,input_tgt_cloud,0,255,0);

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	pcl::PointCloud<PointType> align_cloud;
	struct timeval t0,t1;
	double caltime;
	std::string output_name;
	std::string icp_name;
	std::string pile_name;
	std::cout<<"What is the name of combined file name?"<<std::endl;
	cin>>output_name;
	icp_name=output_name+"_icp.pcd";
	pile_name=output_name+"_pile.pcd";
	gettimeofday(&t0,NULL);
	icp.setInputSource(src_cloud);
	icp.setInputTarget(tgt_cloud);
	//icp.setMaximumIterations(1000);
	icp.align(align_cloud);

	pcl::PointCloud<PointType>::Ptr cmb_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr pile_cloud(new pcl::PointCloud<PointType>);

	//be careful for pointer
	combine_points(cmb_cloud,tgt_cloud,align_cloud);
	combine_points(pile_cloud,tgt_cloud,*src_cloud);
	
	gettimeofday(&t1,NULL);

	std::cout<<"has converged:"<<icp.hasConverged()<<" socre:"<<icp.getFitnessScore()<<std::endl;
	std::cout << "converged file size:" << cmb_cloud->points.size() << std::endl;
	std::cout<<"trabsformation matrix -> "<<std::endl;
	std::cout << icp.getFinalTransformation() <<std::endl;

	caltime=(t1.tv_sec-t0.tv_sec)+1e-6*(t1.tv_usec-t0.tv_usec);
	std::cout<<"calculation time : "<<caltime<<std::endl;
	
	pcl::io::savePCDFileBinary(icp_name,*cmb_cloud);
	pcl::io::savePCDFileBinary(pile_name,*pile_cloud);

	//[test] save pcd file ascii
	//std::string ascii_src_name="ascii_"+source_name;
	//std::string ascii_tgt_name="ascii_"+target_name;
	//std::string ascii_icp_name="ascii_"+icp_name;
	//pcl::io::savePCDFileASCII(ascii_src_name,*src_cloud);
	//pcl::io::savePCDFileASCII(ascii_tgt_name,*tgt_cloud);
	//pcl::io::savePCDFileASCII(ascii_icp_name,*cmb_cloud);
	
	std::cout<<"Saved"<<std::endl;
	
	//ros::spin();

	return 0;
}
