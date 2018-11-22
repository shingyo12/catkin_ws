#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <geometry_msgs/Pose.h>

void subscribeCallback(const geometry_msgs::Pose& gps)
{
	ROS_INFO_STREAM("subscribe callback");
	ROS_INFO("x:%lf y:%lf",gps.position.x,gps.position.y);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "clt_test");
	ros::NodeHandle nh0;
	
	ros::ServiceClient testClient = nh0.serviceClient<std_srvs::Empty>("get_gps");
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;

	ros::Rate rate(1);

	ros::Subscriber sub = nh0.subscribe("gps_data",1,&subscribeCallback);
	ros::Publisher pub = nh0.advertise<std_msgs::Float64>("start_gps",1);
	int i=0,j=0;
	std_msgs::Float64 flt_msg;
	
	while(ros::ok()){
	        double d;
		ROS_INFO("count:%d",i);
		if(i>15){
			ROS_INFO_STREAM("send request");
			bool result = testClient.call(req, resp);
			if(result){
				ROS_INFO_STREAM("Recieve response");
       			}else{
				ROS_INFO_STREAM("Error");
			}
			i=0;
		}
		if(i==5){
			ROS_INFO_STREAM("send start message");
			flt_msg.data=j;
			pub.publish(flt_msg);
			j++;
		}
		i++;
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
