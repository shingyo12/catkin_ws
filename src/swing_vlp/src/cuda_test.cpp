#include <ros/ros.h>
#include "array.h"

int main(int argc, char **argv){
	ros::init(argc,argv,"cuda_test");
	ros::NodeHandle nh;

	//while(nh.ok()){
		array_main();
		//sleep(1);
		//}
	return 0;
}
