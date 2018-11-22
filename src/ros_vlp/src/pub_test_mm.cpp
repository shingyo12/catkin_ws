#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"pub_test");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Float64>("test_return",1);
	ros::Rate rate(1);
	static double d=0.0;
	
	while(ros::ok()){
		std_msgs::Float64 msg;
		d++;
		msg.data=d;
		ROS_INFO_STREAM(msg.data);
		pub.publish(msg);
		rate.sleep();
	}
	return 0;
}
