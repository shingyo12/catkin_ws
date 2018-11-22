#include <ros/ros.h>
#include <std_srvs/Empy.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "clt_test");
	ros::NodeHandle nh;
	ros::ServiceClient testClient = nh.serviceClient<std_srvs::Empty>("test");
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;
	bool result = testClient.call(req, resp);
	if(rsult) ROS_INFO_STREAM("Recieve response");
	else ROS_INFO_STREAM("Error");
}

	
