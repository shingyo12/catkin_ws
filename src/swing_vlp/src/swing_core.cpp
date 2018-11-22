#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/Joy.h>

// class definition
class TeleopJoy{
	public:
        //constructor
		TeleopJoy();

	//hiding 
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh;
    		int rec_start, rec_stop;
    		int ang_boost, init_pos;
    		int rot_bias, flx_bias;
    		ros::Publisher  cmd_pub_;
    		ros::Subscriber joy_sub_;
};

//constructor
TeleopJoy::TeleopJoy():
	        //button
		rec_start(3),
		rec_stop(2),
		ang_boost(1),
		init_pos(0),
		//axis
		rot_bias(4),
		flx_bias(5)   //initialize
{
	/*nh.param("axis_linear"  , vel_linear , vel_linear); //set parameter
	nh.param("axis_angular" , vel_angular, vel_angular);
	nh.param("angular_boost" , ang_boost, ang_boost);
	nh.param("linear_boost" , lin_boost, lin_boost);
	nh.param("scale_angular", a_scale_, a_scale_);
	nh.param("scale_linear" , l_scale_, l_scale_);*/

	//subsucriber,use template
        //this pointa is object pointa
	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);
	cmd_pub_ = nh.advertise<std_msgs::Int8MultiArray>("cmd_ctrl", 1);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	std_msgs::Int8MultiArray cmd_ctrl;
	        //[0]->start
	        //[1]->stop
	        //[2]->ang_boost
	        //[3]->init_pos
	        //[4]->rot_bias
	        //[5]->flx_pos
	cmd_ctrl.data.resize(6);
	//button
	if(joy->buttons[rec_start] > 0){
		//std::cout<<"rec_start"<<std::endl;
		cmd_ctrl.data[0] = 1;
		//std::cout<<"[0]"<<cmd_ctrl.data[0]<<std::endl;
	}else{
		cmd_ctrl.data[0] = 0;
	}
	if(joy->buttons[rec_stop] > 0){
		//std::cout<<"rec_stop"<<std::endl;
		cmd_ctrl.data[1] = 1;
	}else{
		cmd_ctrl.data[1] = 0;
	}
	if(joy->buttons[ang_boost] > 0){
		//std::cout<<"ang_boost"<<std::endl;
		cmd_ctrl.data[2] = 1;
	}else{
		cmd_ctrl.data[2] = 0;
	}
	if(joy->buttons[init_pos] > 0){
		//std::cout<<"init_pos"<<std::endl;
		cmd_ctrl.data[3] = 1;
	}else{
		cmd_ctrl.data[3] = 0;
	}
	//axes
	if(joy->axes[rot_bias] > 0){
		cmd_ctrl.data[4] = 1;
	}else if(joy->axes[rot_bias] < 0){
		cmd_ctrl.data[4] = -1;
	}else{
		cmd_ctrl.data[4] = 0;
	}
	if(joy->axes[flx_bias] > 0){
		cmd_ctrl.data[5] = 1;
	}else if(joy->axes[flx_bias] < 0){
		cmd_ctrl.data[5] = -1;
	}else{
		cmd_ctrl.data[5] = 0;
	}
	std::cout<<"[0]"<<(int)cmd_ctrl.data[0]<<" [1]"<<(int)cmd_ctrl.data[1]<<" [2]"<<(int)cmd_ctrl.data[2]
		 <<" [3]"<<(int)cmd_ctrl.data[3]<<" [4]"<<(int)cmd_ctrl.data[4]<<" [5]"<<(int)cmd_ctrl.data[5]<<std::endl;
	//printf("[0] %d\n",cmd_ctrl.data[0]);
	cmd_pub_.publish(cmd_ctrl);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "swing_core");
	TeleopJoy teleop_joy;

	ros::spin();
	return 0;
}
