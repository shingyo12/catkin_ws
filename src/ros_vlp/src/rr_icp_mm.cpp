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

typedef pcl::PointXYZI PointType;

int main(int argc, char **argv){
	struct timeval ta0,ta1;
	int p,q;
	ros::init(argc, argv, "rr_icp");
	ros::NodeHandle nh;
	gettimeofday(&ta0,NULL);
	for(int m=0;m<5;m++){
		//for(int i=0;i<5;i++){
		//for(int j=0;j<5;j++){
				pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>);
				pcl::PointCloud<PointType>::Ptr tgt_cloud(new pcl::PointCloud<PointType>);
				std::string source_name;
				std::string target_name;
				std::stringstream ss_sc,ss_tg;
				if(m>0){
					p=m*10;
				}else{
					p=m;
				}
				
				q=(m+1)*10;
				ss_sc<<"vox/no"<<0<<"_x"<<q<<"_y0.pcd";
				ss_tg<<"vox/no"<<0<<"_x"<<0<<"_y0.pcd";
				target_name=ss_tg.str();
				source_name=ss_sc.str();
				std::cout<<"target file -> "<<target_name<<std::endl;
				std::cout<<"source file -> "<<source_name<<std::endl;
				//std::cout<<"What do you use Source file?"<<std::endl;
				//cin>>source_name;
				//std::cout<<"What do you use Target file?"<<std::endl;
				//cin>>target_name;

				pcl::io::loadPCDFile(source_name,*src_cloud);
				pcl::io::loadPCDFile(target_name,*tgt_cloud);

				//icp_align(src_cloud,tgt_cloud);
				std::cout << "source file size:" << src_cloud->points.size() << std::endl;
				std::cout << "target file size:" << tgt_cloud->points.size() << std::endl;

				pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
				pcl::PointCloud<PointType> align_cloud;
				struct timeval t0,t1;
				double caltime;
				std::string icp_name;
				//std::cout<<"What is the name of combined file name?"<<std::endl;

				std::stringstream ss_icp;
				ss_icp<<"vox/s"<<0<<0<<"_"<<q<<"_"<<0<<".pcd";
				icp_name=ss_icp.str();
				
				//cin>>icp_name;
				gettimeofday(&t0,NULL);
				icp.setInputSource(src_cloud);
				icp.setInputTarget(tgt_cloud);
				icp.align(align_cloud);

				pcl::PointCloud<PointType>::Ptr cmb_cloud(new pcl::PointCloud<PointType>);

				cmb_cloud->width=tgt_cloud->width+align_cloud.width;
				cmb_cloud->height=align_cloud.height;
				cmb_cloud->is_dense=true;
				//cmb_cloud->points.resize(align_cloud.points.size()+tgt_cloud->points.size());
				for(int r=0;r<tgt_cloud->points.size();++r){	 
					cmb_cloud->points.push_back(tgt_cloud->points[r]);
				}
				for(int r=0;r<align_cloud.points.size();++r){	 
					cmb_cloud->points.push_back(align_cloud.points[r]);
				}
	
				gettimeofday(&t1,NULL);

				std::cout<<"has converged:"<<icp.hasConverged()<<" socre:"<<icp.getFitnessScore()<<std::endl;
				std::cout << "converged file size:" << cmb_cloud->points.size() << std::endl;
				std::cout<<"trabsformation matrix -> "<<std::endl;
				std::cout << icp.getFinalTransformation() <<std::endl;

				caltime=(t1.tv_sec-t0.tv_sec)+1e-6*(t1.tv_usec-t0.tv_usec);
				std::cout<<"calculation time : "<<caltime<<std::endl;
	
				pcl::io::savePCDFileBinary(icp_name,*cmb_cloud);
				std::cout<<"Saved"<<std::endl;
				std::cout<<"aligned file -> "<<icp_name<<std::endl;
				std::cout<<"\n"<<std::endl;
	
				//ros::spin();
				//}
				//	}
	}
	gettimeofday(&ta1,NULL);
	double caltime_total=(ta1.tv_sec-ta0.tv_sec)+1e-6*(ta1.tv_usec-ta0.tv_usec);
	std::cout<<"\n"<<"total time : "<<caltime_total<<std::endl; 
	return 0;
}
