#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <time.h>
#include <cmath>

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
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointType;
#define VOX_SIZE 0.1

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Viewer" ) );
// Point Cloud Color Hndler
pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
void init_viz(){
	//Point cloud Vector
	pcl::PointCloud<PointType>::Ptr vizClouds(new pcl::PointCloud<PointType>);
	//pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

	// PCL Visualizer
    	viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
	viewer->addCoordinateSystem( 5.0  ,"coordinate" );
    	viewer->initCameraParameters();
    	viewer->setCameraPosition( 25.0, 25.0, 500.0, 0.0, 0.0, 0.0, 0 );

        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType> > color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "z" ) );

        handler = color_handler;
}

void cloud_viz(pcl::PointCloud<PointType>::Ptr vizClouds){
	handler->setInputCloud(vizClouds);
        if( !viewer->updatePointCloud( vizClouds, *handler, "trcloud" ) ){
		viewer->addPointCloud( vizClouds, *handler, "trcloud" );
	}

	// Update Viewer
        viewer->spinOnce();
}

void cloud_viz(pcl::PointCloud<PointType> fnc_cloud){
	pcl::PointCloud<PointType>::Ptr vizClouds;
	*vizClouds = fnc_cloud;
	handler->setInputCloud(vizClouds);
        if( !viewer->updatePointCloud( vizClouds, *handler, "trcloud" ) ){
		viewer->addPointCloud( vizClouds, *handler, "trcloud" );
	}

	// Update Viewer
        viewer->spin();
}

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

void computeAnglesFlomMatrix(const Eigen::Matrix4d & R, double & roll, double & pitch, double & yaw){
	double threshold = 0.001;
	printf("abs %lf\n",std::abs((double)R(2,1) - 1.0));
	if(std::abs((double)R(2,1) - 1.0) < threshold){
		roll = M_PI/2;
		pitch = 0;
		yaw = atan2((double)R(2,1), (double)R(0.0));
	}else if(std::abs((double)R(2,1) + 1.0) < threshold){
		roll = -M_PI/2;
		pitch = 0;
		yaw = atan2((double)R(1,0), (double)R(0.0));
	}else{
		roll = asin((double)R(2,1));
		pitch = atan2(-(double)R(2,0), (double)R(2,2));
		yaw = atan2(-(double)R(0,1), (double)R(1,1));
	}
}
int main(int argc, char **argv){

	ros::init(argc, argv, "manual_icp");
	ros::NodeHandle nh;

	init_viz();
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_src_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	int itr_num;
	double t_yaw,t_pitch,t_roll;
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
	pcl::PointCloud<PointType> filtered_cloud[itr_num];
	pcl::PointCloud<PointType>::Ptr tmp_input_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr tmp_align_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr tmp_filter_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType> tmp_transform_cloud;
	struct timeval t0,t1,t2,t3;
	double caltime;
	std::string output_name;
	std::string icp_name;
	std::string pile_name;
	std::string fil_name;
	std::cout<<"What is the name of combined file name?"<<std::endl;
	cin>>output_name;
	icp_name=output_name+"_icp.pcd";
	pile_name=output_name+"_pile.pcd";
	fil_name=output_name+"_fil.pcd";
	gettimeofday(&t0,NULL);

	pcl::PointCloud<PointType>::Ptr pile_cloud(new pcl::PointCloud<PointType>);
	pile_cloud->is_dense=true;
	for(int k=0;k<itr_num;k++){
		for(int r=0;r<input_cloud[k].points.size();++r){	 
			pile_cloud->points.push_back(input_cloud[k].points[r]);
		}
	}

	for(int j=0;j<itr_num;j++){
		*tmp_filter_cloud=input_cloud[j];
		//create z filtering object
		pcl::PassThrough<PointType> pass_z;
		pass_z.setInputCloud (tmp_filter_cloud);
		pass_z.setFilterFieldName ("z");
		pass_z.setFilterLimits (2,200); //2600
		//pass.setFilterLimitsNegative(true);
		pass_z.filter (filtered_cloud[j]);
		std::cout<<"point size : "<<input_cloud[j].points.size()<<" -> "<<filtered_cloud[j].points.size()<<std::endl;
		//cloud_viz(filtered_cloud[j]);
	}

	align_cloud[0]=filtered_cloud[0];
	for(int k=0;k<itr_num-1;k++){
		gettimeofday(&t2,NULL);
		*tmp_input_cloud=filtered_cloud[k+1];
		*tmp_align_cloud=align_cloud[k];
		icp.setInputSource(tmp_input_cloud);
	        icp.setInputTarget(tmp_align_cloud);
		//icp.setInputSource(tmp_align_cloud);
	        //icp.setInputTarget(tmp_input_cloud);
		//icp.setMaxCorrespondenceDistance(0.01);
		//icp.setMaximumIterations(50);
		cloud_viz(tmp_input_cloud);
		icp.align(align_cloud[k+1]);
		//cloud_viz(tmp_align_cloud);
		printf("[filterwd] icp target %d\n",k);
		gettimeofday(&t3,NULL);
		caltime=(t3.tv_sec-t2.tv_sec)+1e-6*(t3.tv_usec-t2.tv_usec);
		std::cout<<k<<" [filtered] calculation time : "<<caltime<<std::endl;
		std::cout<<"has converged:"<<icp.hasConverged()<<" socre:"<<icp.getFitnessScore()<<std::endl;
		std::cout<<"[filtered] transformation matrix -> "<<std::endl;
		std::cout<<icp.getFinalTransformation() <<std::endl;
		//for(int m=k+1;m<itr_num;m++){
		//tmp_transform_cloud=input_cloud[k+1];
		//pcl::transformPointCloud(input_cloud[k+1],tmp_transform_cloud,icp.getFinalTransformation());

		//matrix
		Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
		transformation_matrix = icp.getFinalTransformation ().cast<double>();
		computeAnglesFlomMatrix(transformation_matrix,t_roll,t_pitch,t_yaw);
		printf("roll:%lf pitch:%lf yaw:%lf\n",t_roll,t_pitch,t_yaw);

		//transform
		double cosY=cos(t_yaw);
		double sinY=sin(t_yaw);
		//for(int m=k+1;m<itr_num;m++){
			for(int n=0;n<input_cloud[k+1].points.size();n++){
				double tmp_x = input_cloud[k+1].points[n].x;
				double tmp_y = input_cloud[k+1].points[n].y;
				input_cloud[k+1].points[n].x = cosY*tmp_x - sinY*tmp_y;
				input_cloud[k+1].points[n].y = sinY*tmp_x + cosY*tmp_y;
			}
			//}
		//std::cout<<(double)t_yaw<<" "<<(double)t_pitch<<" "<<(double)t_roll<std::endl;
			//input_cloud[m]=tmp_transform_cloud;
			//pcl::transformPointCloud(filtered_cloud[m],tmp_transform_cloud,icp.getFinalTransformation());
			//input_cloud[m]=tmp_transform_cloud;
			//}
		
	}

	pcl::PointCloud<PointType>::Ptr fil_cloud(new pcl::PointCloud<PointType>);
	fil_cloud->is_dense=true;
	for(int k=0;k<itr_num;k++){
		for(int r=0;r<align_cloud[k].points.size();++r){	 
			fil_cloud->points.push_back(align_cloud[k].points[r]);
		}
	}

	align_cloud[0]=input_cloud[0];
	for(int k=0;k<itr_num-1;k++){
		gettimeofday(&t2,NULL);
		*tmp_input_cloud=input_cloud[k+1];
		*tmp_align_cloud=align_cloud[k];
		icp.setInputSource(tmp_input_cloud);
	        icp.setInputTarget(tmp_align_cloud);
		//icp.setInputSource(tmp_align_cloud);
	        //icp.setInputTarget(tmp_input_cloud);
		//icp.setMaxCorrespondenceDistance(0.01);
		//icp.setMaximumIterations(50);
		cloud_viz(tmp_input_cloud);
		icp.align(align_cloud[k+1]);
		printf("icp target %d\n",k);
		gettimeofday(&t3,NULL);
		caltime=(t3.tv_sec-t2.tv_sec)+1e-6*(t3.tv_usec-t2.tv_usec);
		std::cout<<k<<" calculation time : "<<caltime<<std::endl;
		std::cout<<"has converged:"<<icp.hasConverged()<<" socre:"<<icp.getFitnessScore()<<std::endl;
		std::cout<<"transformation matrix -> "<<std::endl;
		std::cout<<icp.getFinalTransformation() <<std::endl;
		/*for(int m=k+1;m<itr_num;m++){
			pcl::transformPointCloud(input_cloud[m],tmp_transform_cloud,icp.getFinalTransformation());
			input_cloud[m]=tmp_transform_cloud;
			}*/
	}

	pcl::PointCloud<PointType>::Ptr cmb_cloud(new pcl::PointCloud<PointType>);
	cmb_cloud->is_dense=true;
	for(int k=0;k<itr_num;k++){
		for(int r=0;r<align_cloud[k].points.size();++r){	 
			cmb_cloud->points.push_back(align_cloud[k].points[r]);
		}
	}
	
	gettimeofday(&t1,NULL);

	caltime=(t1.tv_sec-t0.tv_sec)+1e-6*(t1.tv_usec-t0.tv_usec);
	std::cout<<"all alculation time : "<<caltime<<std::endl;

	std::cout << "icp file size:" << cmb_cloud->points.size() << std::endl;
	std::cout << "piled file size:" << pile_cloud->points.size() << std::endl;
	
	pcl::io::savePCDFileBinary(icp_name,*cmb_cloud);
	pcl::io::savePCDFileBinary(pile_name,*pile_cloud);
	pcl::io::savePCDFileBinary(fil_name,*fil_cloud);
	
	std::cout<<"Saved"<<std::endl;
	
	//ros::spin();

	return 0;
}
