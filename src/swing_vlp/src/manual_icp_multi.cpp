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
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointType;
#define VOX_SIZE 0.1

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

pcl::PointCloud<pcl::PointXYZRGB> points_i2rgb(pcl::PointCloud<pcl::PointXYZI> i_cloud,int r,int g,int b){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	rgb_cloud->width=i_cloud.width;
   	rgb_cloud->height=i_cloud.height;
   	rgb_cloud->is_dense=true;
   	rgb_cloud->points.resize(i_cloud.width * i_cloud.height);
	for(int r=0;r<rgb_cloud->points.size();++r){	 
		rgb_cloud->points[r].x=i_cloud.points[r].x;
		rgb_cloud->points[r].y=i_cloud.points[r].y;
		rgb_cloud->points[r].z=i_cloud.points[r].z;
		rgb_cloud->points[r].r=r;
		rgb_cloud->points[r].g=g;
		rgb_cloud->points[r].b=b;
        }
	return *rgb_cloud;
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

	int itr_num;
	printf("how many pcd files ");
	scanf("%d",&itr_num);
	std::string pcd_name[itr_num];
	pcl::PointCloud<pcl::PointXYZRGB> input_cloud[itr_num];
	//pcl::PointCloud<pcl::PointXYZRGB> input_cloud1;
	std::cout<<"input file name"<<std::endl;
	for(int i=0;i<itr_num;i++){
		cin>>pcd_name[i];
		pcl::io::loadPCDFile(pcd_name[i],*input_src_cloud);
		std::cout <<i<<" "<<pcd_name[i]<<" file size:" << input_src_cloud->points.size() << std::endl;
		//pcl::PointCloud<pcl::PointXYZI> voxel_cloud=voxel_grid(input_src_cloud);
		//pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxel_cloud(&voxel_cloud);
		if(i%2==0){
			input_cloud[i]=points_i2rgb(voxel_grid(input_src_cloud),255,0,0);
		}else{
			input_cloud[i]=points_i2rgb(voxel_grid(input_src_cloud),0,255,0);
		}
	}

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	pcl::PointCloud<PointType> align_cloud[itr_num];
	pcl::PointCloud<PointType>::Ptr tmp_input_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr tmp_align_cloud(new pcl::PointCloud<PointType>);
	struct timeval t0,t1,t2,t3;
	double caltime;
	std::string output_name;
	std::string icp_name;
	std::string pile_name;
	std::cout<<"What is the name of combined file name?"<<std::endl;
	cin>>output_name;
	icp_name=output_name+"_icp.pcd";
	pile_name=output_name+"_pile.pcd";
	gettimeofday(&t0,NULL);

	align_cloud[0]=input_cloud[0];
	for(int k=0;k<itr_num-1;k++){
		gettimeofday(&t2,NULL);
		*tmp_input_cloud=input_cloud[k+1];
		*tmp_align_cloud=align_cloud[k];
		icp.setInputSource(tmp_input_cloud);
	        icp.setInputTarget(tmp_align_cloud);
		//icp.setMaxCorrespondenceDistance(15);
		icp.setMaximumIterations(40);
		icp.align(align_cloud[k+1]);
		printf("icp target %d\n",k);
		gettimeofday(&t3,NULL);
		caltime=(t3.tv_sec-t2.tv_sec)+1e-6*(t3.tv_usec-t2.tv_usec);
		std::cout<<k<<" calculation time : "<<caltime<<std::endl;
		std::cout<<"has converged:"<<icp.hasConverged()<<" socre:"<<icp.getFitnessScore()<<std::endl;
		std::cout<<"trabsformation matrix -> "<<std::endl;
		std::cout<<icp.getFinalTransformation() <<std::endl;
	}

	pcl::PointCloud<PointType>::Ptr cmb_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr pile_cloud(new pcl::PointCloud<PointType>);

	cmb_cloud->is_dense=true;
	pile_cloud->is_dense=true;
	for(int k=0;k<itr_num;k++){
		for(int r=0;r<align_cloud[k].points.size();++r){	 
			cmb_cloud->points.push_back(align_cloud[k].points[r]);
		}
		for(int r=0;r<input_cloud[k].points.size();++r){	 
			pile_cloud->points.push_back(input_cloud[k].points[r]);
		}
	}
	
	gettimeofday(&t1,NULL);

	caltime=(t1.tv_sec-t0.tv_sec)+1e-6*(t1.tv_usec-t0.tv_usec);
	std::cout<<"all alculation time : "<<caltime<<std::endl;

	std::cout << "icp file size:" << cmb_cloud->points.size() << std::endl;
	std::cout << "piled file size:" << pile_cloud->points.size() << std::endl;
	
	pcl::io::savePCDFileBinary(icp_name,*cmb_cloud);
	pcl::io::savePCDFileBinary(pile_name,*pile_cloud);
	
	std::cout<<"Saved"<<std::endl;
	
	//ros::spin();

	return 0;
}
