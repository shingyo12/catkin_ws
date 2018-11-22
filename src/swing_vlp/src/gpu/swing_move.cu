#include <stdio.h>
#include <stdlib.h>
#include "array.h"
#include "swing_move.h"

//#define Flexion_Angle 1.0
#define Z_offset 1350

__global__ void coodinate_transform(float *g_x, float *g_y, float *g_z,
				            float *xs, float *ys, float *zs, int n,
				                double rotangle, double d_rotangle, double y, double p, double r){
	//int i = blockIdx.x * blockDim.x + threadIdx.x;
//arr3[i] = sin(arr2[i]);
        
}

void swing_move(float *rect_x, float *rect_y, float *rect_z, int n,
		                double b, double rotangle, double d_rotangle,
		                                         double y, double p, double r){
	//a is rotation angle of axis (rad)
	int i, j;
	double a;
	double sa,ca;  //sin(a) and cos(a)
	double d_lo = 0.025 + 0.0377;  //distance : flexion point to laser origin (mm)
	double cosY,cosP,cosR,sinY,sinP,sinR;
	float *d_xs,*d_ys,*d_zs,  *d_x,*d_y,*d_z;
	float xs,ys,zs;
	size_t n_byte = n*16*sizeof(float);

	//cpu malloc
	//xs = (float *)malloc(n_byte);
	//ys = (float *)malloc(n_byte);
	//zs = (float *)malloc(n_byte);

	//gpu malloc
	cudaMalloc((float**)&d_x, n_byte);
        cudaMalloc((float**)&d_y, n_byte);
	cudaMalloc((float**)&d_z, n_byte);
	cudaMalloc((float**)&d_xs, n_byte);
	cudaMalloc((float**)&d_ys, n_byte);
	cudaMalloc((float**)&d_zs, n_byte);

	//cpu memory copy to gpu memory
	cudaMemcpy(d_x, rect_x, n_byte, cudaMemcpyHostToDevice);
	cudaMemcpy(d_y, rect_y, n_byte, cudaMemcpyHostToDevice);
	cudaMemcpy(d_z, rect_z, n_byte, cudaMemcpyHostToDevice);

//	coodinate_transform<<<1,16>>>(d_x,d_y,d_z,d_xs,d_ys,d_zs,n,rotangle,d_rotangle,y,p,r);
	for(i = 0; i < n; i++){
		rotangle += d_rotangle;
		coodinate_transform<<<1,16>>>(d_x,d_y,d_z,d_xs,d_ys,d_zs,n,rotangle,d_rotangle,y,p,r);
		//std::printf("%lf\n",axis_rotangle);
		a = rotangle * M_PI/180;
		sa=sin(a);
		ca=cos(a);
		for(j = 0; j < 16; j++){
			xs = d_x[i*j];
			ys = d_y[i*j];
			zs = d_z[i*j];
			d_x[i*j] = (sa*sa+ca*ca)*xs + b*ca*zs + d_lo*b*ca;
			d_y[i*j] = (sa*sa+ca*ca)*ys + b*sa*zs + d_lo*b*sa;
			d_z[i*j] = -b*ca*xs -b*sa*ys + zs + d_lo + Z_offset;

			//using imu
		        cosY=cos(y);
			cosP=cos(p);
			cosR=cos(r);
			sinY=sin(y);
			sinP=sin(p);
			sinR=sin(r);

			xs = d_x[i*j];
			ys = d_y[i*j];
			zs = d_z[i*j];

			d_x[i*j]=cosP*cosR*xs + (sinY*sinP*cosR-cosY*sinR)*ys + (sinY*sinR+cosY*sinP*cosR)*zs;
			d_y[i*j]=cosP*sinR*xs + (sinY*sinP*sinR+cosY*cosR)*ys + (-sinY*cosR+cosY*sinP*sinR)*zs;
			d_z[i*j]=-sinP*xs + sinY*cosP*ys + cosY*cosP*zs;

		}
		//std::cout<<"  rotation_angle "<<axis_rotangle;
	}

	//gpu memory copy to cpu memory
	cudaMemcpy(rect_x, d_x, n_byte, cudaMemcpyDeviceToHost);
	cudaMemcpy(rect_y, d_y, n_byte, cudaMemcpyDeviceToHost);
	cudaMemcpy(rect_z, d_z, n_byte, cudaMemcpyDeviceToHost);

	cudaFree(d_x);
	cudaFree(d_y);
	cudaFree(d_z);
	cudaFree(d_xs);
	cudaFree(d_ys);
	cudaFree(d_zs);
}

/*void swing_move(struct point *pnt, double a,double b,double d_lo,struct imu){
	double  *d_x, *d_y, *d_z;
	size_t n_byte = N * sizeof(double);

	arr1 = (float *)malloc(n_byte);
	arr2 = (float *)malloc(n_byte);
	arr3 = (float *)malloc(n_byte);

	initialize_array(arr1, n_byte);
	initialize_array(arr2, n_byte);
	initialize_array(arr3, n_byte);

	printf("start cudaMalloc\n");
	cudaMalloc((void**)&d_arr1, N);
	cudaMalloc((void**)&d_arr2, N);
	cudaMalloc((void**)&d_arr3, N);
	printf("finish sudaMallloc\n");

	printf("start cudaMemcpy\n");
	cudaMemcpy(d_arr1, arr1, n_byte, cudaMemcpyHostToDevice);
	cudaMemcpy(d_arr2, arr2, n_byte, cudaMemcpyHostToDevice);
	cudaMemcpy(d_arr3, arr3, n_byte, cudaMemcpyHostToDevice);
	printf("fnish cudamemcpy\n");

	printf("start kernel funcion\n");
	sum_of_array<<<1, 16>>>(d_arr1, d_arr2, d_arr3, n_byte);
	printf("finish kernel function\n");
	cudaMemcpy(arr3, d_arr3, n_byte, cudaMemcpyDeviceToHost);
	}*/

/*void swing_coodinate_trans(struct dots *q, int pnum_start, int pnum_end, double rotangle, double d_rotangle){
	double  *d_x, *d_y, *d_z;
	int n = (pnum_end - pnum_start) * 16;
	//size_t n_byte = n * sizeof(double);
	
	int i, j ,k = 0;
	double a;  //rotation angle of axis (rad)
	double sa,ca;  //sin(a) and cos(a)
	double s_ang = 0;
	double b = (Flexion_Angle+s_ang) * M_PI/180;  //flexion angle of axis (rad)
	double d_lo = 25 + 37.7;  //distance : flexion point to laser origin (mm)
	//i is Block loop val.
	for(i = pnum_start; i < pnum_end; i++){
		rotangle += d_rotangle;
		//std::printf("%lf\n",axis_rotangle);
		a = rotangle * M_PI/180;
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

	}*/
	
