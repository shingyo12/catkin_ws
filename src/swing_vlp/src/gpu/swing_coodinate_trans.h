#ifndef SWING_COODINATE_TRANS_H
#define SWING_COODINATE_TRANS_H

struct point{
	double x;
	double y;
	double z;
};

//imu posiion data
struct rpy{
       double r=0;
       double p=0;
       double y=0;
};

//extern void swing_coodinate_trans(struct dots *q, int pnum_start, int pnum_end, double rotangle, double d_rotangle);

#endif
