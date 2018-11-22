#include <iostream>
#include <string>
#include <vector>

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
#include <pcl/filters/passthrough.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/filters/voxel_grid.h>

#define VOX_SIZE 100

struct map{
	int x;
	int y;
	int z_max;
	int z_min;
	int type;

	bool operator<( const map& right ) const {
		return x==right.x ? y < right.y : x < right.x;
	}
};

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

int main(int argc, char** argv){
	//file name seting
	std::string in_fname,out_fname;
	if(argc==3){
		std::stringstream ss_in,ss_out;
		ss_in<<argv[1];
		in_fname=ss_in.str();
		std::cout<<in_fname<<" is opened"<<std::endl;

		ss_out<<argv[2];
		out_fname=ss_out.str();
		std::cout<<"output to "<<out_fname<<std::endl;
	}else{
		std::cout<<"Please input PCD file name"<<std::endl;
		return 0;
	}
	sensor_msgs::PointCloud2 PointCloud;
	typedef pcl::PointXYZI PointType;
	pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

	//ROS initialize
	ros::init(argc, argv, "map_make");
	ros::NodeHandle nh;

	//loading input file
	pcl::io::loadPCDFile(in_fname,*input_cloud);
	std::cout<<"input point cloud size : "<<input_cloud->points.size()<<std::endl;

	//apply voxel grid filter to input file 
	*filtered_cloud=voxel_grid(input_cloud);
	printf("voxel grid filter process is done\n");

	//search points and make grid map
	std::vector<map> grid;
	int grid_x,grid_y;
	int grid_z_max,grid_z_min;
	float cm_x,cm_y,cm_z;
	std::vector<int> previous;
	int pre=0;
	int max_x,min_x,max_y,min_y;
	printf("making grid map .... \n");
	cm_x=filtered_cloud->points[0].x*0.01;
	cm_y=filtered_cloud->points[0].y*0.01;
	max_x=min_x=(int)cm_x;
	max_y=min_y=(int)cm_y;
	for(int i=0; i<filtered_cloud->points.size(); i++){
		if(filtered_cloud->points[i].z*0.01>1 ){
			cm_x=filtered_cloud->points[i].x*0.01;
			cm_y=filtered_cloud->points[i].y*0.01;
			cm_z=filtered_cloud->points[i].z*0.01;
			grid_x=(int)cm_x;
			grid_y=(int)cm_y;
			grid_z_max=(int)cm_z;
			grid_z_min=(int)cm_z;
			for(int k=i+1; k<filtered_cloud->points.size(); k++){
				if(filtered_cloud->points[k].z*0.01>1 ){
					cm_x=filtered_cloud->points[k].x*0.01;
					cm_y=filtered_cloud->points[k].y*0.01;
					cm_z=filtered_cloud->points[k].z*0.01;
					if(grid_x == (int)cm_x && grid_y == (int)cm_y){
						if(grid_z_max > (int)cm_z){
							grid_z_max=(int)cm_z;
						}
						if(grid_z_min < (int)cm_z){
							grid_z_min=(int)cm_z;
						}
						previous.push_back(k);
					}
				}
			}
			for(int num:previous){
				//pre=0;
				if(num==i)
					pre=1;
			}
			if(pre!=1){
				grid.push_back({grid_x,grid_y,grid_z_max,grid_z_min,0});
				if(grid_z_max!=grid_z_min){
					std::cout<<grid_x<<"  "<<grid_y<<"  "<<grid_z_min<<"  "<<grid_z_max<<std::endl;
				}
			}
			if(max_x<grid_x)
				max_x=grid_x;
			if(min_x>grid_x)
				min_x=grid_x;
			if(max_y<grid_y)
				max_y=grid_y;
			if(min_y<grid_y)
				min_y=grid_y;
			pre=0;
		}
	}
	std::cout<<"grid size : "<<grid.size()<<std::endl;

	sort(grid.begin(), grid.end());

	ofstream outputmap(out_fname+".map");
	outputmap<<"# "<<max_x+std::abs(min_x)<<" "<<max_y+std::abs(min_y)<<" "<<std::abs(min_x)<<" "
		                                             <<std::abs(min_y)<<" 0.0 0.0 0.0"<<std::endl;
	for(int i=0;i<grid.size();i++){
		outputmap<<grid[i].x<<" "<<grid[i].y<<" "<<grid[i].z_min<<" "<<grid[i].z_max<<std::endl;
	}
	outputmap.close();

	ofstream outputwp(out_fname+".wp");
	outputwp<<"0.000000 0.000000 0.000000"<<std::endl;
	outputwp<<"0.000000 0.000000 6.000000 0 1.500000"<<std::endl;
	outputwp.close();
	
	std::cout<<"Saved"<<std::endl;
	//ros::spin();
}
