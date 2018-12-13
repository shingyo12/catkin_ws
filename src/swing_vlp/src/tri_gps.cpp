#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

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

#define tty_dev "/dev/ttyACM0"
#define BUFF_SIZE 8

#define M_LAT2M 1854.388056
#define M_LNG2M 1519.319352

struct position {
	double x;
	double y;
	double z;
};

struct gps_data{
	double lon;	//longitude
	double lat;	//latitude
	double alt;	//altitude

	double x;
	double y;
	double z;
	double direction;
};

int start_gps=0;
int ref_gps=0;

class plane_coordinate{
private:
	int form;	//formation of plane coordinate
	double B_0, B_1, B_2, B_3, B_4, B_5, B_6, B_7, B_8, B_9;
	double origin_latitude;
	double origin_longitude;
	//double now_latitude;
	//double now_longitude;
	double a;	//the long radius of the earth
        double b;	//the short radius of the earth
	double S,S_0;
	double phi,phi_0;	//latitude(radians)
	double lambda,lambda_0,d_lambda;	//longitude(radisns)
	double m_0;	//scale factor at the origin of coordinate system
	double F;	//reciprocal of oblateness
	double X,Y;
	double e,e_d;
	double N;
	
	double meridian_arc(double phi);
	double vertical_radius(double phi);
public:
	plane_coordinate(int form);
	double plane_x(double now_latitude,double now_longitude);
	double plane_y(double now_latitude,double now_longitude);
};

plane_coordinate::plane_coordinate(int form) : a(6378137.00),b(6356752.31),m_0(0.9999),F(298.257222101) {
	//select formation
	if(form==1){
		origin_latitude = 33;
		origin_longitude= 129+30/60;
	}else if(form==2){
		origin_latitude = 33;
		origin_longitude= 131+30/60;
        }else if(form==3){
		origin_latitude = 36;
		origin_longitude= 132+10/60;
	}else if(form==4){
		origin_latitude = 33;
		origin_longitude= 133+30/60;
        }else if(form==5){
		origin_latitude = 36;
		origin_longitude= 134+20/60;
	}else if(form==6){
		origin_latitude = 36;
		origin_longitude= 136;
	}else if(form==7){
		origin_latitude = 36;
		origin_longitude= 137+10/60;
	}else if(form==8){
		origin_latitude = 36;
		origin_longitude= 138+30/60;
	}else if(form==10){
		origin_latitude = 40;
		origin_longitude= 140+50/60;
	}else if(form==11){
		origin_latitude = 44;
		origin_longitude= 140+15/60;
	}else if(form==12){
		origin_latitude = 44;
		origin_longitude= 142+15/60;
	}else if(form==13){
		origin_latitude = 44;
		origin_longitude= 142;
	}else if(form==14){
		origin_latitude = 26;
		origin_longitude= 142;
	}else if(form==15){
		origin_latitude = 26;
		origin_longitude= 127;
	}else if(form==16){
		origin_latitude = 26;
		origin_longitude= 124;
	}else if(form==17){
		origin_latitude = 26;
		origin_longitude= 131;
	}else if(form==18){
		origin_latitude = 20;
		origin_longitude= 136;
	}else if(form==19){
		origin_latitude = 26;
		origin_longitude= 154;
        }else{
		origin_latitude = 36;
		origin_longitude= 129+50/60;
	}

	//initalization for conversion
	e   = sqrt(a*a-b*b/a*a);
	e_d = sqrt(a*a-b*b/b*b);
	
	double A = 1 + pow(e,2)*3/4 + pow(e,4)*45/64 + pow(e,6)*175/256 + pow(e,8)*11025/16384 + pow(e,10)*43659/65536
			+ pow(e,12)*693693/1048576 + pow(e,14)*19324395/29360128 + pow(e,16)*4927697775/7516192768;
	double B = pow(e,2)*3/4 + pow(e,4)*15/16 + pow(e,6)*525/512 + pow(e,8)*2205/2048 + pow(e,10)*72765/65536 + pow(e,12)*297297/266144
			+ pow(e,14)*135270135/117440512 + pow(e,16)*547521975/469762048;
	double C = pow(e,4)*15/64 + pow(e,6)*105/256 + pow(e,8)*2205/4096 + pow(e,10)*10395/16384 + pow(e,12)*1486485/2097152
			+ pow(e,14)*45090045/58720256 + pow(e,16)*766530765/939524096;
        double D = pow(e,6)*35/512 + pow(e,8)*315/16384 + pow(e,10)*31185/131072 + pow(e,12)*165165/524288 + pow(e,14)*4099095/29360128
			+ pow(e,16)*209053845/469762048;
	double E = pow(e,8)*315/16384 + pow(e,10)*3465/65536 + pow(e,12)*99099/1048576 + pow(e,14)*4099095/29360128 + pow(e,16)*348423075/1879048192;
	double F = pow(e,10)*693/131072 + pow(e,12)*9009/524288 + pow(e,14)*4099095/117440512 * pow(e,16)*26801775/939524096;
	double G = pow(e,12)*3003/2097152 + pow(e,14)*315315/58720256 + pow(e,16)*11486475/939524096;
	double H = pow(e,14)*45045/117440512 + pow(e,16)*765765/469762048;
	double I = pow(e,16)*765765/7516192768;

	B_0 = a*(1-exp(2));
	B_1 = B_0*A;
	B_2 = B_0*(-B/2);
	B_3 = B_0*(C/4);
	B_4 = B_0*(-D/6);
	B_5 = B_0*(E/8);
	B_6 = B_0*(-F/10);
	B_7 = B_0*(G/12);
        B_8 = B_0*(-H/14);
	B_9 = B_0*(I/16);

	phi_0 = origin_latitude*M_PI/180;
	lambda_0 = origin_longitude*M_PI/180;
	S_0 = B_1*phi_0 + B_2*sin(2*phi_0) + B_3*sin(4*phi_0) + B_4*sin(6*phi_0) + B_5*sin(8*phi_0) + B_6*sin(10*phi_0)
		+ B_7*sin(12*phi_0) + B_8*sin(14*phi_0) + B_9*sin(16*phi_0);
}

double plane_coordinate::meridian_arc(double phi){
	//phi = lat*M_PI/180;
	double arc= B_1*phi + B_2*sin(2*phi) + B_3*sin(4*phi) + B_4*sin(6*phi) + B_5*sin(8*phi) + B_6*sin(10*phi)
		+ B_7*sin(12*phi) + B_8*sin(14*phi) + B_9*sin(16*phi);
	return arc;
}

double plane_coordinate::vertical_radius(double phi){
	//phi = lat*M_PI/180;
	//double v_rad=a/(sqrt(1-pow(e*sin(lat*M_PI/180))));
	return a/(sqrt(1-pow(e*sin(phi),2)));
}

double plane_coordinate::plane_x(double now_latitude,double now_longitude){
	//printf("plane_x\n");
	phi = now_latitude*M_PI/180;
	lambda = now_longitude*M_PI/180;
	S = meridian_arc(phi);
	N = vertical_radius(phi);
	double t = tan(phi);
	d_lambda = lambda-lambda_0;
	double sqr_eta = pow((e_d*cos(phi)),2);
	
	X = (S-S_0) + N*pow(cos(phi),2)*t*pow(d_lambda,2)/2 + N*pow(cos(phi),4)*t*(5-t*t+9*sqr_eta+4*pow(sqr_eta,2))*pow(d_lambda,4)/24
		- N*pow(cos(phi),6)*t*(-61 -58*t*t-pow(t,4) -270*sqr_eta + 330*pow(t,2)*pow(sqr_eta,2))*pow(d_lambda,6)/720
		- N*pow(cos(phi),8)*t*(-1385 +3111*t*t -543*pow(t,4) +pow(t,6))*pow(d_lambda,8)/40320;
	X = X*m_0;
	return X;
}

double plane_coordinate::plane_y(double now_latitude,double now_longitude){
	//printf("plane_y\n");
	phi = now_latitude*M_PI/180;
	lambda = now_longitude*M_PI/180;
	N = vertical_radius(phi);
	double t = tan(phi);
	d_lambda = lambda-lambda_0;
	double sqr_eta = pow((e_d*cos(phi)),2);
	
	Y = N*cos(phi)*d_lambda - N*pow(cos(phi),3)*(-1-t*t-sqr_eta)*pow(d_lambda,3)/6
		- N*pow(cos(phi),5)*(-5 -18*t*t-pow(t,4) -14*sqr_eta + 58*t*t*sqr_eta)*pow(d_lambda,5)/120
		- N*pow(cos(phi),7)*(-61 +479*t*t -179*pow(t,4) +pow(t,6))*pow(d_lambda,78)/5040;
	Y = Y*m_0;
	return Y;
}



//ros node main
int config_port(void){
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

void gl2lc(std::string lglt,int ps_flg,position& sensor_pos,std_msgs::Float64MultiArray& lat_lon){
                    //lglt : longitude and latitute
	printf("gl2lc\n");
	static double ref_latitude,ref_longitude,dis,ref_elevation;
	static int ref_cnt=0;
	gps_data gps_f,gps_l,gps_r;
	position reference_pos;
	size_t str_size=lglt.length();
	std::cout<<lglt;
	std::cout<<str_size<<std::endl;
	gps_f.lat = std::stod(lglt.substr(17,10));
	gps_f.lon = std::stod(lglt.substr(30,11));
	gps_f.alt = std::stod(lglt.substr(54,5));
	gps_r.lat = std::stod(lglt.substr(17,10));
	gps_r.lon = std::stod(lglt.substr(30,11));
	gps_r.alt = std::stod(lglt.substr(54,5));
	gps_l.lat = std::stod(lglt.substr(17,10));
	gps_l.lon = std::stod(lglt.substr(30,11));
	gps_l.alt = std::stod(lglt.substr(54,5));
	int mode  = (int)std::stod(lglt.substr(44,1));


}

void JsCallback(const  std_msgs::Float32MultiArray& cmd_ctrl){
	//printf("callback\n");
	std::cout<<"[0]"<<(int)cmd_ctrl.data[0]<<" [1]"<<(int)cmd_ctrl.data[1]<<" [2]"<<(int)cmd_ctrl.data[2]
		 <<" [3]"<<(int)cmd_ctrl.data[3]<<" [4]"<<(int)cmd_ctrl.data[4]<<" [5]"<<(int)cmd_ctrl.data[5]<<std::endl;
	if((int)cmd_ctrl.data[0] == 1){
		start_gps = 1;
		if((int)cmd_ctrl.data[3] == 1){
			ref_gps = 1;
		}
	}
	if((int)cmd_ctrl.data[1] == 1){
		start_gps = 0;
		ref_gps = 0;
	}
}

int main(int argc, char **argv){
	int fd=config_port();	
	int n,i,j=0;
  	int result;
        int len;
  	char buf[BUFF_SIZE];
        std::string head;
	size_t head_size;
        position sensor_pos;

	//ROS initialize
	ros::init(argc, argv, "usb_gps_js0");
	ros::NodeHandle nh;
	//ROS : making publisher
	ros::Publisher pub0 = nh.advertise<geometry_msgs::Pose>("gps_0",1);
	ros::Publisher pub1 = nh.advertise<std_msgs::Float64MultiArray>("lat_lon_0",1);
	ros::Subscriber sub = nh.subscribe("cmd_ctrl",1,JsCallback);

	geometry_msgs::Pose gps;
	std_msgs::Float64MultiArray lat_lon;
	lat_lon.data.resize(2);
	
        while(ros::ok()) {
		//printf("reading serial\n");
		len = read(fd, buf, BUFF_SIZE);
		//printf("read serial\n");
		//printf("ros::ok");
		if (0<len){
			for(i=0;i<len;i++){
				//printf("len\n");
				if(buf[i]=='$'){
					//printf("$\n");
					if(head[2]=='N' && head[3]=='G' && head[4]=='G' && head[5]=='A'){
						printf("call function\n");
						if(head.length()>70 && head.length()<100){
							gl2lc(head,ref_gps,sensor_pos,lat_lon);
						}
					}
					gps.position.x=sensor_pos.x;
					gps.position.y=sensor_pos.y;
					gps.position.z=sensor_pos.z;
					pub0.publish(gps);
					pub1.publish(lat_lon);
					head.erase(0);
					//printf("%lf %lf\n",lat_lon.data[0],lat_lon.data[1]);
				}
				head+=buf[i];
				//j++;
			}	
		}
		ros::spinOnce();
	}
	return 0;
}
