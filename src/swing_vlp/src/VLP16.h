/************************************************************************/
/* Velodyne HDL-32E header file						*/
/* This files contains HDL-32E calibration setting data.		*/
/*									*/
/* note:2012.4.24 by #M50 1lt Shinya Ohkawa				*/
/* remake for VLP-16 by #60 Kouhei Matsumoto		*/
/************************************************************************/

#include <time.h>

struct LRF {
	
	struct 		timespec t;
	unsigned int	rot[12];
	unsigned int	dis[12][32];
	unsigned int	ref[12][32]; 
	unsigned int	gps;

};

struct ppLRF {	
	struct 				timespec t;
	std::vector<unsigned int>		rot;
	std::vector<std::vector<unsigned int>>	dis;
	std::vector<std::vector<unsigned int>>	ref; 
	unsigned int			gps;

};

//class dots : point data x,y,z,i,collor
struct dots {
	std::vector<std::vector<float>> tX;
	std::vector<std::vector<float>> tY;
	std::vector<std::vector<float>> tZ;
	std::vector<std::vector<float>> tIntensity;
	std::vector<std::vector<float>> tCollor;
	};

struct POS {

	double		rad[12];
	double		x[12][32];
	double		y[12][32];
	double		z[12][32];	

};

double aov[32]= { -15.0,1.00,-13.00,3.00,-11.00,5.00,-9.00,7.00,-7.00,9.00,-5.00,11.00,-3.00,13.00,-1.00,15.00,
				 -15.0,1.00,-13.00,3.00,-11.00,5.00,-9.00,7.00,-7.00,9.00,-5.00,11.00,-3.00,13.00,-1.00,15.00 };

#define Flexion_Angle 1.088357//1.26066

#define FV01 -15.00
#define FV02 1.00
#define FV03 -13.00
#define FV04 3.00
#define FV05 -11.00
#define FV06 5.00
#define FV07 -9.00
#define FV08 7.00
#define FV09 -7.00
#define FV10 9.00
#define FV11 -5.00
#define FV12 11.00
#define FV13 -3.00
#define FV14 13.00
#define FV15 -1.00
#define FV16 15.00
#define FV17 -15.00
#define FV18 1.00
#define FV19 -13.00
#define FV20 3.00
#define FV21 -11.00
#define FV22 5.00
#define FV23 -9.00
#define FV24 7.00
#define FV25 -7.00
#define FV26 9.00
#define FV27 -5.00
#define FV28 11.00
#define FV29 -3.00
#define FV30 13.00
#define FV31 -1.00
#define FV32 15.00
