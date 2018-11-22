#include <iostream>
#include <string>
#include <vector>

#include <cmath>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>

#include "VLP16.h"
#include "Dataset.h"

#include <unistd.h> // UNIX standard function definations 
#include <fcntl.h>  // File control definations 
#include <termios.h>// POSIX terminal control definations 

#include <asm/ioctls.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#define S_SET 0 
#define E_SET 32
//#define Flexion_Angle 1.065//1.26066

//#define Flexion_Angle 1.0
#define Z_offset 1350

//LRF pich direction angular
static double Psin[32];
static double Pcos[32];

//use data stract
static struct dataset *d;
static double **raw;
static int nraw;
static double **v;

//double swing_cycle;
double axis_d_rotangle;
double axis_rotangle = 360;

double AOV[32];
int    idx[32] = {0};

//imu posiion data
struct rpy{
       double r=0;
       double p=0;
       double y=0;
};
rpy imu;

void init_LRF(){
	//make pich direction sin & cos val from caliration table.	
	Psin[0] = sin(FV01 * M_PI / 180); 
	Psin[1] = sin(FV02 * M_PI / 180); 
	Psin[2] = sin(FV03 * M_PI / 180); 
	Psin[3] = sin(FV04 * M_PI / 180); 
	Psin[4] = sin(FV05 * M_PI / 180); 
	Psin[5] = sin(FV06 * M_PI / 180); 
	Psin[6] = sin(FV07 * M_PI / 180); 
	Psin[7] = sin(FV08 * M_PI / 180); 
	Psin[8] = sin(FV09 * M_PI / 180); 
	Psin[9] = sin(FV10 * M_PI / 180); 
	Psin[10] = sin(FV11 * M_PI / 180); 
	Psin[11] = sin(FV12 * M_PI / 180); 
	Psin[12] = sin(FV13 * M_PI / 180); 
	Psin[13] = sin(FV14 * M_PI / 180); 
	Psin[14] = sin(FV15 * M_PI / 180); 
	Psin[15] = sin(FV16 * M_PI / 180); 
	Psin[16] = sin(FV17 * M_PI / 180); 
	Psin[17] = sin(FV18 * M_PI / 180); 
	Psin[18] = sin(FV19 * M_PI / 180); 
	Psin[19] = sin(FV20 * M_PI / 180); 
	Psin[20] = sin(FV21 * M_PI / 180); 
	Psin[21] = sin(FV22 * M_PI / 180); 
	Psin[22] = sin(FV23 * M_PI / 180); 
	Psin[23] = sin(FV24 * M_PI / 180); 
	Psin[24] = sin(FV25 * M_PI / 180); 
	Psin[25] = sin(FV26 * M_PI / 180); 
	Psin[26] = sin(FV27 * M_PI / 180); 
	Psin[27] = sin(FV28 * M_PI / 180); 
	Psin[28] = sin(FV29 * M_PI / 180); 
	Psin[29] = sin(FV30 * M_PI / 180); 
	Psin[30] = sin(FV31 * M_PI / 180); 
	Psin[31] = sin(FV32 * M_PI / 180); 
	
	Pcos[0] = cos(FV01 * M_PI / 180); 
	Pcos[1] = cos(FV02 * M_PI / 180); 
	Pcos[2] = cos(FV03 * M_PI / 180); 
	Pcos[3] = cos(FV04 * M_PI / 180); 
	Pcos[4] = cos(FV05 * M_PI / 180); 
	Pcos[5] = cos(FV06 * M_PI / 180); 
	Pcos[6] = cos(FV07 * M_PI / 180); 
	Pcos[7] = cos(FV08 * M_PI / 180); 
	Pcos[8] = cos(FV09 * M_PI / 180); 
	Pcos[9] = cos(FV10 * M_PI / 180); 
	Pcos[10] = cos(FV11 * M_PI / 180); 
	Pcos[11] = cos(FV12 * M_PI / 180); 
	Pcos[12] = cos(FV13 * M_PI / 180); 
	Pcos[13] = cos(FV14 * M_PI / 180); 
	Pcos[14] = cos(FV15 * M_PI / 180); 
	Pcos[15] = cos(FV16 * M_PI / 180); 
	Pcos[16] = cos(FV17 * M_PI / 180); 
	Pcos[17] = cos(FV18 * M_PI / 180); 
	Pcos[18] = cos(FV19 * M_PI / 180); 
	Pcos[19] = cos(FV20 * M_PI / 180); 
	Pcos[20] = cos(FV21 * M_PI / 180); 
	Pcos[21] = cos(FV22 * M_PI / 180); 
	Pcos[22] = cos(FV23 * M_PI / 180); 
	Pcos[23] = cos(FV24 * M_PI / 180); 
	Pcos[24] = cos(FV25 * M_PI / 180); 
	Pcos[25] = cos(FV26 * M_PI / 180); 
	Pcos[26] = cos(FV27 * M_PI / 180); 
	Pcos[27] = cos(FV28 * M_PI / 180); 
	Pcos[28] = cos(FV29 * M_PI / 180); 
	Pcos[29] = cos(FV30 * M_PI / 180); 
	Pcos[30] = cos(FV31 * M_PI / 180); 
	Pcos[31] = cos(FV32 * M_PI / 180); 
	
	// change of laser		
	int	i;
	for(i = 0; i < 32; i++){
		if(i % 2 == 0)
			idx[i] = i / 2;
		else
			idx[i] = i / 2 + 16;
	}

	for(i=0;i<32;i++) {
		AOV[i]=  aov[i]*M_PI/180.0;
	}
}
static int test=0;
void messageCallBack(const std_msgs::Float64& msg){
	//std::printf(" callback\n");
	//swing_cycle=msg.data;
	
	axis_rotangle = 80;
	axis_d_rotangle=-0.1*(0.2/360)*(360/msg.data);
	//ROS_INFO_STREAM(axis_d_rotangle);
	//std::printf("callback %d %lf\n",test,axis_d_rotangle);
	test++;
}

void imu_callback(const  geometry_msgs::Quaternion& qtn_msgs){
	tf::Quaternion qtn(qtn_msgs.x,qtn_msgs.y,qtn_msgs.z,qtn_msgs.w);
	//std::cout<<qtn<<std::endl;
	tf::Matrix3x3 m(qtn);
	m.getRPY(imu.r,imu.p,imu.y);
	std::printf("r:%lf p:%lf y:%lf\n",imu.r,imu.p,imu.y);
	//std::printf("%lf %lf %lf %lf\n",qtn_msgs.x,qtn_msgs.y,qtn_msgs.z,qtn_msgs.w);
}

/*void imu_coodinate_trans(double& x, double& y,double& z){
	double cosY=cos(imu.y);
	double cosP=cos(imu.p);
	double cosR=cos(imu.r);
	double sinY=sin(imu.y);
	double sinP=sin(imu.p);
	double sinR=sin(imu.r);

	double tx=x;
	double ty=y;
	double tz=z;

	//x=cosP*cosR*tx + (sinY*sinP*cosR-cosY*sinR)*ty + (sinY*sinR+cosY*sinP*cosR)*tz;
	x=tx*2;
	y=cosP*sinR*tx + (sinY*sinP*sinR+cosY*cosR)*ty + (-sinY*cosR+cosY*sinP*sinR)*tz;
	z=-sinP*tx + sinY*cosP*ty + cosY*cosP*tz;

	//std::printf("r:%lf p:%lf y:%lf\n",imu.r,imu.p,imu.y);
	}*/

void swing_coodinate_trans(struct dots *q, int pnum_start, int pnum_end){
	//std::printf("swing coordinate trans %d\n",dn);
	int i, j ,k = 0;
	double a;  //rotation angle of axis (rad)
	double sa,ca;  //sin(a) and cos(a)
	double s_ang = 0;
	double b = (Flexion_Angle+s_ang) * M_PI/180;  //flexion angle of axis (rad)
	double d_lo = 25 + 37.7;  //distance : flexion point to laser origin (mm)
	//i is Block loop val.
	for(i = pnum_start; i < pnum_end; i++){
		axis_rotangle += axis_d_rotangle;
		//std::printf("%lf\n",axis_rotangle);
		a = axis_rotangle * M_PI/180;
		sa=sin(a);
		ca=cos(a);
		for(j = 0; j < 16; j++){
			double xs = q->tX[i][j];
			double ys = q->tY[i][j];
			double zs = q->tZ[i][j];
			q->tX[i][j] = (sa*sa+ca*ca)*xs + b*ca*zs + d_lo*b*ca;
			q->tY[i][j] = (sa*sa+ca*ca)*ys + b*sa*zs + d_lo*b*sa;
			q->tZ[i][j] = -b*ca*xs -b*sa*ys + zs + d_lo + Z_offset;

			//using imu
			double cosY=cos(imu.y);
			double cosP=cos(imu.p);
			double cosR=cos(imu.r);
			double sinY=sin(imu.y);
			double sinP=sin(imu.p);
			double sinR=sin(imu.r);

			double tx=q->tX[i][j];
			double ty=q->tY[i][j];
			double tz=q->tZ[i][j];

			q->tX[i][j]=cosP*cosR*tx + (sinY*sinP*cosR-cosY*sinR)*ty + (sinY*sinR+cosY*sinP*cosR)*tz;
			q->tY[i][j]=cosP*sinR*tx + (sinY*sinP*sinR+cosY*cosR)*ty + (-sinY*cosR+cosY*sinP*sinR)*tz;
			q->tZ[i][j]=-sinP*tx + sinY*cosP*ty + cosY*cosP*tz;

		}
		//std::cout<<"  rotation_angle "<<axis_rotangle;
	}
	//std::cout<<"ct size "<<size-2<<endl;
	//std::cout<<"  rotangle "<<a<<std::endl;
}

void coodinate_trans(struct ppLRF *p, struct dots *q, const int dn, int& r){
	//std::printf(" coordinate trans %d\n",dn);
	int i, j ,k = 0;
	int pnum_start, pnum_end;
	pnum_start=r;
	q->tX.resize(dn*2);
   	q->tY.resize(dn*2);
   	q->tZ.resize(dn*2);
   	q->tIntensity.resize(dn*2);
	q->tCollor.resize(dn*2);
	//std::cout<<"size of p.dis "<<sizeof(p)<<endl;
	//i is Block loop val.
	for(i = 0; i < 12; i++){
		double rad = (double)p->rot[i] / 100 * 3.141592 / 180;
		double rad2 = ((double)p->rot[i] / 100 + 0.2)* 3.141592 / 180;
		double Ysin = sin(rad);
		double Ycos = cos(rad);
		double Ysin2 = sin(rad2);
		double Ycos2 = cos(rad2);
		//j is Line loop val.
		//p.dis is LRF distance data.
		//p.ref is LRF reflection data.
		//Psin & Pcos is Pich direction LRF offset angle.
		q->tX[r].resize(16);
   		q->tY[r].resize(16);
   		q->tZ[r].resize(16);
   		q->tIntensity[r].resize(16);
		q->tCollor[r].resize(16);

		q->tX[r+1].resize(16);
   		q->tY[r+1].resize(16);
   		q->tZ[r+1].resize(16);
   		q->tIntensity[r+1].resize(16);
		q->tCollor[r+1].resize(16);
		//std::printf("Block\n");
		for(j = S_SET; j < E_SET; j++){
			//std::cout<<"for"<<endl;
			if(j<16){
				//std::printf(" array0 %d\n",r);
				q->tX[r][j] = ((double)p->dis[i][j] * Pcos[j] * Ysin) * 2;
				q->tY[r][j] = ((double)p->dis[i][j] * Pcos[j] * Ycos) * 2;
				q->tZ[r][j] = ((double)p->dis[i][j] * Psin[j]) * 2 ;
				q->tIntensity[r][j] = p->ref[i][j];
				//q->tCollor[r][j] = Psin[j];
			}else{
				//std::printf(" array2 %d\n",r+1);
				q->tX[r+1][j-16] = ((double)p->dis[i][j] * Pcos[j] * Ysin2) * 2;
				q->tY[r+1][j-16] = ((double)p->dis[i][j] * Pcos[j] * Ycos2) * 2;
				q->tZ[r+1][j-16] = ((double)p->dis[i][j] * Psin[j]) * 2 ;
				q->tIntensity[r+1][j-16] = p->ref[i][j];
				//q->tCollor[r+1][j-16] = Psin[j];
			}
		}
		//std::cout<<"r:"<<r+1<<std::endl;
		r = r+2;
		pnum_end=r;
	}
	//Coodinate transformation about swing mechanism
	swing_coodinate_trans( q, pnum_start, pnum_end );
}

void receiveLRF(void *arg){
	//std::printf(" recieveLRF\n");
	// read init table.
	init_LRF();
	// point cloud val.
	d = (struct dataset *)arg;
	raw 	= d->p;
	nraw 	= d->np;
	unsigned int	fno = 0;
	
	d->nv = 0;

	// packet ctrl val.
	int     plen;
	int     Blocks, Lines;

	// network val.
	int     sock;
	struct	sockaddr_in addr;
	unsigned char	buf[1206];

	//file name check.
	//if user no input file name. then mode flag shift 1.
	FILE	*fp;
	char	filename[100];
	int 	mode = 0;

	//make socket
	sock = socket(AF_INET, SOCK_DGRAM, 0);

	//socket setting
	addr.sin_family = AF_INET;
	addr.sin_port = htons(2368);
	addr.sin_addr.s_addr = INADDR_ANY;
	//std::printf("socketsetting\n");
	//socket bind
	bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	//std::printf("socketbind\n");
	//first dummy read
	recv(sock, buf, sizeof(buf), 0);
	//std::printf("firstdummy\n");

	//velodyne data
	ppLRF p;
	//dots
	dots q;
	
	//velodine data number in one round
	unsigned int dn = 0;

	//rotate angle
	double azimuth,pazimuth=0;
	int rtflg = 0;

	//dots total
	int r=0;

	//point cloud type in ROS
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("cycle",1,&messageCallBack);
	ros::Publisher pub  = nh.advertise<sensor_msgs::PointCloud2>("point2",10);
	ros::Subscriber sub1 = nh.subscribe("imu_pose",10,imu_callback);
	sensor_msgs::PointCloud2 point2;
	std_msgs::Float64 msg;

	pcl::PointCloud<pcl::PointXYZI> trcloud;

	while(ros::ok()){
		//printf("P_cycle %lf\n",P_cycle);
		//data recive
		int chk = sizeof(buf);
		while(chk != 0){
			plen = recv(sock, buf + sizeof(buf) - chk, chk, 0);
			chk -= plen; 
		}

		//data recive clock time
		clock_gettime(CLOCK_REALTIME, &p.t);

		//make rotation, distance & intensity data
		for(Blocks = 0; Blocks < 12; Blocks++){
			//std::printf(" recieve-block\n");
			//resize vector
			p.rot.resize(12);
			p.dis.resize(12);
			p.ref.resize(12);
			
			//check start identifier 
			if(buf[Blocks * 100] != 0xFF || buf[Blocks * 100 + 1] != 0xEE)
				printf("Data Start identifier Error!!\n");

			//make rotational position
			//Rot is senser angle.
			p.rot[Blocks] = buf[Blocks * 100 + 2] + (buf[Blocks * 100 + 3] << 8);	

			p.dis[Blocks].resize(32);
			p.ref[Blocks].resize(32);
			for(Lines = 0; Lines < 32; Lines++) {
				//Dis is distance.
				//Ref is intensity.
				p.dis[Blocks][Lines] = 	buf[Blocks * 100 + Lines * 3 + 4] 
						 	  + (buf[Blocks * 100 + Lines* 3 + 5] << 8);
				p.ref[Blocks][Lines] = 	buf[Blocks * 100 + Lines * 3 + 6];
			}
			//cout<<"cn:"<<cn<<" dn:"<<dn<<endl;
			dn++;
		}
		//coodinate transform from raw data to 3D XYZ position data.
		coodinate_trans( &p, &q, dn, r);
		
		//have laser made One roud?
		azimuth = buf[0 * 100 + 2] + (buf[0 * 100 + 3] << 8);
		if(azimuth<pazimuth){
			rtflg = 1;
			//std::cout<<"rotate "<<"azimuth:"<<azimuth<<" pazimuth:"<<pazimuth<<std::endl;
		}
		pazimuth=azimuth;

		if(rtflg==1){
			//input to new point cloud data
			trcloud.width=dn*2;
   			trcloud.height=16;
   			trcloud.is_dense=true;
   			trcloud.points.resize(trcloud.width*trcloud.height);
   			int j=0;
			//std::cout<<"tr size "<<dn*2<<std::endl;
   			/*for(int r=0;r<dn*2-2;r++){
				//std::printf(" for r=%d dn=%d\n",r,dn-1);
	    			for(int k=0;k<16;k++){	 
					trcloud.points[j].x=q.tX[r][k];
		 			trcloud.points[j].y=q.tY[r][k];
		 			trcloud.points[j].z=q.tZ[r][k];
		 			trcloud.points[j].intensity=q.tIntensity[r][k];
		 			j++;
	      			}
	 		}*/
			//std::cout<<"size:"<<j<<std::endl;
			for(int r=0;r<dn*2-2;++r){
	    			for(int k=0;k<16;k++){	 
					trcloud.at(r,k).x=q.tX[r][k];
		 			trcloud.at(r,k).y=q.tY[r][k];
		 			trcloud.at(r,k).z=q.tZ[r][k];
		 			trcloud.at(r,k).intensity=q.tIntensity[r][k];
		 			//j++;
	      			}
	 		}
			
			pcl::toROSMsg(trcloud,point2);
			pub.publish(point2);
			rtflg=0;
			dn=0;
			r=0;
			
		}
		ros::spinOnce();
	}
}

int main(int argc, char *argv[]){	
	std::printf(" Welcome : VLP-16 to point cloud data.\n");
	
	//loop count val.
	int i,j;

	//use only offline mode file pointer.
	FILE	*fp;
	FILE	*fq;
	struct LRF p;

	//ready initial points val. 
	struct dataset dp;

	ros::init(argc, argv, "vlp_pcd");
	receiveLRF(&dp);

	return 0;
}




