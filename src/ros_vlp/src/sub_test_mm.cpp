#include <ros/ros.h>
#include <std_msgs/Float64.h>

void subscribeCallback(const std_msgs::Float64& msg0)
{
	ROS_INFO_STREAM("subscribe callback");
	ROS_INFO_STREAM(msg0.data);

	//sleep(1);	
}

int main(int argc, char **argv){
	ros::init(argc, argv, "sub_test");
	ros::NodeHandle nh1;
	
	ros::Subscriber sub = nh1.subscribe("test_return",1,&subscribeCallback);
	ros::spin();
	
	return 0;
}

	
