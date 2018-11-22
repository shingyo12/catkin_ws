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

#define S_SET 0 
#define E_SET 32 
#define CloudMax 1

//LRF pich direction angular
static double Psin[32];
static double Pcos[32];

//use data stract
static struct dataset *d;
static double **raw;
static int nraw;
static double **v;

double P_cycle;

double AOV[32];
int	idx[32] = {0};
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
	//end:make pich direction sin & cos val from caliration table.	
	
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

void messageCallBack(const std_msgs::Float64& msg){
	//std::printf(" callback\n");
	ROS_INFO_STREAM(msg.data);
	P_cycle=msg.data;
}

void coodinate_trans(struct ppLRF p, struct dots *q, const int size){
	//std::printf(" coordinate trans %d\n",size);
	int i, j ,k = 0;
	int r = 0;

	q->tX.resize(size*2);
   	q->tY.resize(size*2);
   	q->tZ.resize(size*2);
   	q->tIntensity.resize(size*2);
	q->tCollor.resize(size*2);
	//std::cout<<"size of p.dis "<<sizeof(p)<<endl;
	//i is Block loop val.
	for(i = 0; i < size; i++){
		double rad = (double)p.rot[i] / 100 * 3.141592 / 180;
		double rad2 = ((double)p.rot[i] / 100 + 0.2)* 3.141592 / 180;
		//if(rad==0 || rad>6.283)break;	
		double Ysin = sin(rad);
		double Ycos = cos(rad);
		double Ysin2 = sin(rad2);
		double Ycos2 = cos(rad2);

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
		//std::printf(" array %d\n",r);
		//cout<<"r:"<<r<<" i:"<<i<<endl;
		//std::printf("Block\n");
		for(j = S_SET; j < E_SET; j++){
			//cout<<"r:"<<r<<" j:"<<j<<" i:"<<i<<" dis:"<<p.dis[i][j]<<endl;
			//std::cout<<"for"<<endl;
			if(j<16){
				//std::printf(" array0 %d\n",r);
				q->tX[r][j] = ((double)p.dis[i][j] * Pcos[j] * Ysin) * 2;
				q->tY[r][j] = ((double)p.dis[i][j] * Pcos[j] * Ycos) * 2;
				q->tZ[r][j] = ((double)p.dis[i][j] * Psin[j]) * 2 ;
				q->tIntensity[r][j] = p.ref[i][j];
				//q->tCollor[r][j] = Psin[j];
			}else{
				//std::printf(" array2 %d\n",r+1);
				q->tX[r+1][j-16] = ((double)p.dis[i][j] * Pcos[j] * Ysin2) * 2;
				q->tY[r+1][j-16] = ((double)p.dis[i][j] * Pcos[j] * Ycos2) * 2;
				q->tZ[r+1][j-16] = ((double)p.dis[i][j] * Psin[j]) * 2 ;
				q->tIntensity[r+1][j-16] = p.ref[i][j];
				//q->tCollor[r+1][j-16] = Psin[j];
			}
		}
		//std::cout<<"ct:"<<r+1<<endl;
		r = r+2;
	}
	//std::cout<<"ct size "<<size-2<<endl;
}

void receiveLRF(void *arg)
{
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
	std::vector<ppLRF> p;

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
	//dots
	dots q;
	
	//velodine data number in one round
	unsigned int dn = 0;
	//cloud(in one round) set number
	unsigned int cn = 0;
	p.resize(CloudMax+1);
	//std::printf("resize\n");
	//rotate angle
	double azimuth,pazimuth=0;
	int rtflg = 0;
	int size;

	//point type in ROS
	ros::NodeHandle nh;
	//printf("Sub\n");
	ros::Subscriber sub = nh.subscribe("cycle",1,&messageCallBack);
	//printf("Pub\n");
	ros::Publisher pub  = nh.advertise<sensor_msgs::PointCloud2>("point2",10);
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
		clock_gettime(CLOCK_REALTIME, &p[cn].t);

		//make rotation, distance & intensity data
		for(Blocks = 0; Blocks < 12; Blocks++){
			//std::printf(" recieve-block\n");
			//resize vector
			p[cn].rot.resize(dn+1);
			p[cn].dis.resize(dn+1);
			p[cn].ref.resize(dn+1);
			
			//check start identifier 
			if(buf[Blocks * 100] != 0xFF || buf[Blocks * 100 + 1] != 0xEE)
				printf("Data Start identifier Error!!\n");

			//make rotational position
			//Rot is senser angle.
			p[cn].rot[dn] = buf[Blocks * 100 + 2] + (buf[Blocks * 100 + 3] << 8);	

			p[cn].dis[dn].resize(32);
			p[cn].ref[dn].resize(32);
			for(Lines = 0; Lines < 32; Lines++) {
				//Dis is distance.
				//Ref is intensity.
				p[cn].dis[dn][Lines] = 	buf[Blocks * 100 + Lines * 3 + 4] 
						 	  + (buf[Blocks * 100 + Lines* 3 + 5] << 8);
				p[cn].ref[dn][Lines] = 	buf[Blocks * 100 + Lines * 3 + 6];
			}
			//cout<<"cn:"<<cn<<" dn:"<<dn<<endl;
			dn++;
		}
		p[cn].gps =   buf[1200] + (buf[1201] << 8)	+ (buf[1202] << 16) + (buf[1203] << 24);	

		//have laser made One roud?
		azimuth = buf[0 * 100 + 2] + (buf[0 * 100 + 3] << 8);
		if(azimuth<pazimuth){
			cn++;
			rtflg = 1;
			//cout<<"rotate "<<"azimuth:"<<azimuth<<" pazimuth:"<<pazimuth<<endl;
		}
		pazimuth=azimuth;

		if(rtflg==1){
			size=dn-1;
			//std::cout<<"size:"<<dn<<endl;
			//std::printf(" recieve-sizeof\n");
			//ROS_INFO_STREAM(msg.data);
			//coodinate from raw data to 3D XYZ position data. 
			coodinate_trans(p[cn-1], &q,size);
			//std::printf(" recieve-ct\n");
			//input to new point cloud data
			trcloud.width=size*2;
   			trcloud.height=16;
   			trcloud.is_dense=true;
   			trcloud.points.resize(trcloud.width*trcloud.height);
   			int j=0;
			//cout<<"tr size "<<(size-2)*2-1<<endl;
   			for(int r=0;r<size*2-2;++r){
	    			for(int k=0;k<16;k++){	 
					trcloud.points[j].x=q.tX[r][k];
		 			trcloud.points[j].y=q.tY[r][k];
		 			trcloud.points[j].z=q.tZ[r][k]+1562.7;
		 			trcloud.points[j].intensity=q.tIntensity[r][k];
		 			j++;
	      			}
	 		}
			/*for(int r=0;r<size*2-2;++r){
	    			for(int k=0;k<16;k++){	 
					trcloud.at(r,k).x=q.tX[r][k];
		 			trcloud.at(r,k).y=q.tY[r][k];
		 			trcloud.at(r,k).z=q.tZ[r][k];
		 			trcloud.at(r,k).intensity=q.tIntensity[r][k];
		 			j++;
	      			}
	 		}*/
			pcl::toROSMsg(trcloud,point2);
			//pcl::fromROSMsg(point2,trcloud);
			pub.publish(point2);
			rtflg=0;
			dn=0;
			if(cn==CloudMax)cn=0;
			//cout<<"point size:"<<size*2*16 <<"  j:"<<j<<endl;
			//pcl viewer
			//if(!viewer->wasStopped()){
				//visualiser();
			//}
			
		}
		ros::spinOnce();
	}
}

int main(int argc, char *argv[]){	
	std::printf(" Welcome : VLP-16 to point cloud data.\n");
	//std::printf("%d\n",argc);
	
	//loop count val.
	int i,j;

	//use only offline mode file pointer.
	FILE	*fp;
	FILE	*fq;
	struct LRF p;

	//ready initial points val. 
	struct dataset dp;

	//pcl visualiser setting
	//initVis();
	ros::init(argc, argv, "vlp_pcd_noswing");
	//pthread_t th_rLRF;
	//pthread_create(&th_rLRF, NULL, receiveLRF, &dp);
	receiveLRF(&dp);

	return 0;
}




