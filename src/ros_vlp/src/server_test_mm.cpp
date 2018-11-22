#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>

int flg=0, flg2=0;

bool serviceCallBack(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	ROS_INFO_STREAM("Recieve Request");
	flg=1;
	
	return true;
}

void subscribeCallback(const std_msgs::Float64& msg0)
{
	static int i=1;
	if(i%2 == 1){
		flg2=0;
		ROS_INFO_STREAM("subscribe callback flg2=0");
	}else{
		flg2=1;
		ROS_INFO_STREAM("subscribe callback flg2=1");
	}
	i++;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"srv_test");
	ros::NodeHandle nh;
	ros::ServiceServer srv = nh.advertiseService("gps_data",&serviceCallBack);

	static double d=0.0;
	ros::Publisher pub = nh.advertise<std_msgs::Float64>("test_return",1);
	std_msgs::Float64 msg1;
	ros::Subscriber sub = nh.subscribe("test_flg",1,&subscribeCallback);
	ROS_INFO_STREAM("Server ready");
	ros::Rate rate(1);
	
	while(ros::ok()){
		if(flg2>0){
			if(flg>0){
				msg1.data=d;
				pub.publish(msg1);
				ROS_INFO_STREAM("publish");
				flg=0;
				d=0;
			}
			d++;
			ROS_INFO_STREAM(d);
		}
		ROS_INFO_STREAM("while roop");
		ros::spinOnce();
		rate.sleep();
	}
	
}
