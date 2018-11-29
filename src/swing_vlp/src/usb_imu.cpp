#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <unistd.h> // UNIX standard function definations 
#include <fcntl.h>  // File control definations 
#include <termios.h>// POSIX terminal control definations 

#include <errno.h>
#include <asm/ioctls.h>

#define tty_dev "/dev/ttyUSB2"

char buf[8];

int config_port(void) {
  	int fd;
  	//char*  tty_dev;
 	struct termios tio;
  	//unsigned char buf[50];
	//tty_dev = "/dev/ttyUSB0";
	if ((fd = open(tty_dev, O_RDWR|O_SYNC)) == -1) {
		fprintf(stderr, "failed to open %s with %s\n", tty_dev, strerror(errno));
		return 1;
	}
	ROS_INFO("device open at %d", fd);
	
	tcgetattr(fd, &tio);
	tio.c_cflag = (B230400 | CS8 | CREAD);
	tio.c_iflag = 0;	//IGNPAR;
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 3; // 11
	tcsetattr(fd, TCSANOW, &tio);
	fprintf(stderr, "%s opened at baud rate:230400 bps\n", tty_dev);  // 57600bps

	//ROS_INFO("Serial Test Start\n");
	// synchronize
	while(read(fd, buf, 3) == 3) { 
		//printf("%02x %02x %02x\n",buf[0],buf[1],buf[2]);
		if (buf[0] >= 0x30 || buf[0] <= 0x39  ){
			break;
		}else{
			read(fd, buf, 1);
		}
	}
	ROS_INFO("synchronized");
	return fd;
}

float char2float(std::vector<char> tmp_chr){
	float data;
	int i=0;
	char ch_data[tmp_chr.size()];
	for(char x: tmp_chr){
		ch_data[i]=x;
		i++;
	}
	sscanf(ch_data,"%f",&data);
	return data;
}

/*void get_quaternion_msg(float roll,float pitch,float yaw,geometry_msgs::Quaternion& q){
	tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
	quaternionTFToMsg(quat,q);
	}*/

int main(int argc, char **argv){
	//ROS initialize
	ros::init(argc, argv, "usb_imu");
	ros::NodeHandle nh;
	//ROS : making publisher
	//ros::Publisher pub = nh.advertise<geometry_msgs::Quaternion>("imu_pose",1);
	ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("imu_pose", 1);

	int fd=config_port();
	int n=0,j=0,len;
  	int result;
	double data[7];
	float roll_imu=0,pitch_imu=0,yaw_imu=0,yaw_mag=0,yaw_mag_ref=0,yaw=0;
	int data_cnt=0;

	std::string str;
	std::vector<char> tmp_chr;

	std_msgs::Float64MultiArray imu_pose;
	imu_pose.data.resize(3);
  	
	//geometry_msgs::Quaternion qtn;
	
	while(ros::ok()) {

		len=read(fd, buf, sizeof(buf));
		//std::cout<<buf;
		for(int i=0;i<len;i++){
			//printf("%c",buf[i]);
			if(buf[i]!=' '){
				tmp_chr.push_back(buf[i]);
			}else{
				if(tmp_chr[0]!=' ' && tmp_chr.size()!=0){
					data[j]=char2float(tmp_chr);
					//printf("%d:%lf ",j,data[j]);
					j++;
				}
				tmp_chr.clear();
			}
		}
		//printf("\n");
		if(j>7){
		        //printf("%lf %lf %lf\n",data[1],data[2],data[3]);
			//roll_imu=data[1];
			//pitch_imu=data[2];
			//yaw_imu=data[3];
			/*if(data_cnt==0){
				yaw_imu=data[4];
				yaw_mag_ref=data[4];
			}else{
				yaw_imu=yaw_mag_ref+data[3];
				}
			yaw_mag=data[4];
			//yaw=(yaw_mag + yaw_imu)*0.5;
			yaw=yaw_mag+7.39;

			//roll_imu += 0.1;
			//pitch_imu += 0.0;
			//yaw += 0.0;
			*/

			imu_pose.data[0]=data[3]*M_PI/180; //yaw
			imu_pose.data[1]=data[2]*M_PI/180; //pitch
			imu_pose.data[2]=data[1]*M_PI/180; //roll
			
			data_cnt++;
			printf("yaw:%lf pitch:%lf roll:%lf\n",imu_pose.data[0],imu_pose.data[1],imu_pose.data[2]);
			j=0;
			
			//get_quaternion_msg(-yaw*M_PI/180,-roll_imu*M_PI/180,-pitch_imu*M_PI/180,qtn);
			//printf("%f %f %f %f\n",qtn.x,qtn.y,qtn.z,qtn.w);
			//std::cout<<qtn<<std::endl;
			pub.publish(imu_pose);
		}

		//pub.publish(qtn);
		//ROS_INFO_STREAM(cycle.data);

	}
    return 0;
}




