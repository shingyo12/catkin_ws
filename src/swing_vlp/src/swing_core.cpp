#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// class definition
class TeleopTurtle{
	public:
        //constructor
		TeleopTurtle();

	//hiding 
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh;
    		int vel_linear, vel_angular;
    		int lin_boost, ang_boost;
    		double l_scale_, a_scale_;
    		ros::Publisher  vel_pub_;
    		ros::Subscriber joy_sub_;
};

//constructor
TeleopTurtle::TeleopTurtle(): vel_linear(1), vel_angular(0), a_scale_(3.2),l_scale_(2.0), lin_boost(0), ang_boost(1) //initialize
{
	nh.param("axis_linear"  , vel_linear , vel_linear); //set parameter
	nh.param("axis_angular" , vel_angular, vel_angular);
	nh.param("angular_boost" , ang_boost, ang_boost);
	nh.param("linear_boost" , lin_boost, lin_boost);
	nh.param("scale_angular", a_scale_, a_scale_);
	nh.param("scale_linear" , l_scale_, l_scale_);

	//subsucriber,use template
        //this pointa is object pointa
	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
	vel_pub_ = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	geometry_msgs::Twist twist;
	if (joy->buttons[ang_boost] > 0){
		twist.angular.z = 2 * a_scale_*joy->axes[vel_angular];
	}else{
		twist.angular.z = a_scale_*joy->axes[vel_angular];
	}

	if (joy->buttons[lin_boost] > 0){
		twist.linear.x = 2 * l_scale_*joy->axes[vel_linear];
	}else{
		twist.linear.x = l_scale_*joy->axes[vel_linear];
	}
    
	ROS_INFO_STREAM("(" << joy->axes[vel_linear] << " " << joy->axes[vel_angular] << ")");
	vel_pub_.publish(twist);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "swing_core");
	TeleopTurtle teleop_turtle;

	ros::spin();
	return 0;
}
