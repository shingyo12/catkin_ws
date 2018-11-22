#include <ros/ros.h>
#include <std_msgs/Float64.h>

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

#define tty_dev "/dev/ttyUSB1"

int config_port(void) {
  	int fd;
  	//char*  tty_dev;
 	struct termios tio;
  	unsigned char buf[50];
	//tty_dev = "/dev/ttyUSB0";
	if ((fd = open(tty_dev, O_RDWR|O_SYNC)) == -1) {
		fprintf(stderr, "failed to open %s with %s\n", tty_dev, strerror(errno));
		return 1;
	}
	ROS_INFO("device open at %d", fd);
	
	tcgetattr(fd, &tio);
	tio.c_cflag = (B57600 | CS8 | CREAD);
	tio.c_iflag = 0;	//IGNPAR;
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 3; // 11
	tcsetattr(fd, TCSANOW, &tio);
	fprintf(stderr, "%s opened at baud rate:57600 bps\n", tty_dev);  // 57600bps

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

int main(int argc, char **argv){
	//ROS initialize
	ros::init(argc, argv, "usb_cycle");
	ros::NodeHandle nh;
	//ROS : making publisher
	ros::Publisher pub = nh.advertise<std_msgs::Float64>("cycle",1);

	int fd=config_port();
	
	int n,i;
  	int result;
  	unsigned char buf[50];
  	double start,end,cycle;

	struct timespec t,pt;
	clock_gettime(CLOCK_REALTIME, &t);
	start=(double)t.tv_sec+(double)t.tv_nsec/1000000000;
	
	while(ros::ok()) {

		read(fd, buf, 3);

		buf[3]=0;
	    	clock_gettime(CLOCK_REALTIME, &t);
		end=(double)t.tv_sec+(double)t.tv_nsec/1000000000;
		std_msgs::Float64 cycle;
		cycle.data=end-start;
		pub.publish(cycle);
		ROS_INFO_STREAM(cycle.data);

		start=end;
	}
    return 0;
}




