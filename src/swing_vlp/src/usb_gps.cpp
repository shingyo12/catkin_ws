#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <iostream>
#include <string>
#include <fstream>

#include <unistd.h> // UNIX standard function definations 
#include <fcntl.h>  // File control definations 
#include <termios.h>// POSIX terminal control definations 

#include <errno.h>
#include <asm/ioctls.h>

#define tty_dev "/dev/ttyUSB0"
#define BUFF_SIZE 8

#define M_LAT2M 1854.388056
#define M_LNG2M 1519.319352

struct position {
	double x;
	double y;
};

int config_port(void) {
  	int fd;
 	struct termios tio;
  	char buf[BUFF_SIZE];
	//tty_dev = "/dev/ttyUSB0";
	if ((fd = open(tty_dev, O_RDWR|O_SYNC)) == -1) {
		fprintf(stderr, "failed to open %s with %s\n", tty_dev, strerror(errno));
		return 1;
	}
	printf("device open at %d\n", fd);
	
	tcgetattr(fd, &tio);
        tio.c_cflag = (B9600 | CS8 | CREAD);
	tio.c_iflag = 0;	//IGNPAR;
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 8; // 11
	tcsetattr(fd, TCSANOW, &tio);
	fprintf(stderr, "%s opened at baud rate:9600 bps\n", tty_dev);  // 57600bps

	// synchronize
	while(read(fd, buf, BUFF_SIZE) <= 0);
	printf("synchronized\n");
	return fd;
}

int init_reference_point()
{
	char ans;
	std::cout<<"do you use reference point(y or n)"<<std::endl;
	std::cin>>ans;
	if(ans=='y'){
		return 1;
	}else{
		return 0;
	}   
}

void gl2lc(std::string lglt,int ps_flg,position& sensor_pos){        //lglt : longitude and latitute
	//printf("gl2lc");
	static int cnt=1;
	static double sum_latitude,sum_longitude;
	double ref_latitude,ref_longitude,dis;
	position reference_pos;
	double latitude = std::stod(lglt.substr(17,10));
	double longitude = std::stod(lglt.substr(30,11));
   
	sum_latitude += latitude;
	sum_longitude += longitude;
   
	double ave_latitude = sum_latitude/cnt;
	double ave_longitude = sum_longitude/cnt;
	cnt++;
   
	if(ps_flg == 1){
		//printf("ps_flg==1");
		std::ifstream datafile("reference_point.txt");
		while(datafile>>ref_latitude>>ref_longitude);
		sensor_pos.x=(ave_latitude-ref_latitude)*M_LAT2M;
		sensor_pos.y=(ave_longitude-ref_longitude)*M_LNG2M;
		dis=sqrt(sensor_pos.x*sensor_pos.x+sensor_pos.y*sensor_pos.y);
		printf("x:%lf y:%lf dis:%f  ",sensor_pos.x,sensor_pos.y,dis);
	}else{
		//printf("ps_flg==0");
		std::ofstream fout("reference_point.txt");
		fout.setf(std::ios_base::fixed,std::ios_base::floatfield);
		fout.precision(6);
		fout<<ave_latitude<<"  "<<ave_longitude<<std::endl;
		fout.close();
	
		sensor_pos.x = 0;
		sensor_pos.y = 0;
	}   
	printf("now : %lf %lf   ave : %lf %lf\n",latitude,longitude,ave_latitude,ave_longitude);
}


int main(int argc, char **argv){
	int fd=config_port();	
	int n,i,j=0;
  	int result;
        int len;
  	char buf[BUFF_SIZE];
        std::string head;
        position sensor_pos;
   
        int ans=init_reference_point();

	//ROS initialize
	ros::init(argc, argv, "usb_gps");
	ros::NodeHandle nh;
	//ROS : making publisher
	ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("gps",1);

	geometry_msgs::Pose gps;
	
        while(ros::ok()) {

		len = read(fd, buf, BUFF_SIZE);
		//printf("ros::ok");
		if (0<len){
			for(i=0;i<len;i++){
				if(buf[i]=='$'){
					if(head[2]=='N' && head[3]=='G' && head[4]=='G' && head[5]=='A')
						gl2lc(head,ans,sensor_pos);
					gps.position.x=sensor_pos.x;
					gps.position.y=sensor_pos.y;
					pub.publish(gps);
					head.erase(0);
				}
				head+=buf[i];
				j++;
			}	
		}
	}
	return 0;
}
