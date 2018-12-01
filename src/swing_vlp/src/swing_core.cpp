#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
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
	        int hrz_rot, vrt_rot, hrz_pd, vrt_pd;
	        int zoom_in, zoom_out;
    		ros::Publisher  cmd_pub_;
    		ros::Subscriber joy_sub_;
};

//constructor
TeleopJoy::TeleopJoy():
	        //button
		rec_start(3),
		rec_stop(2),
		ang_boost(7),
		init_pos(0),
		zoom_in(4),
		zoom_out(5),
		//axis
		rot_bias(4),
		flx_bias(5),
		hrz_rot(0),
		vrt_rot(1),
		hrz_pd(2),
		vrt_pd(3)//initialize
		
{
	//subsucriber,use template
        //this pointa is object pointa
	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);
	cmd_pub_ = nh.advertise<std_msgs::Float32MultiArray>("cmd_ctrl", 1);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	std_msgs::Float32MultiArray cmd_ctrl;
	        //[0]->start
	        //[1]->stop
	        //[2]->ang_boost
	        //[3]->init_pos
	        //[4]->rot_bias
	        //[5]->flx_pos
	        //[6]->horizontal_rotation
	        //[7]->vertical_rotation
	        //[8]->horizontal_parallel_displacement
	        //[9]->vertical_parallel_displacement
	        //[10]->zoom_in/out
	cmd_ctrl.data.resize(11);
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
		cmd_ctrl.data[4] = -1;
	}else if(joy->axes[rot_bias] < 0){
		cmd_ctrl.data[4] = 1;
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

	//camera setting
	cmd_ctrl.data[6] = joy->axes[hrz_rot];
	cmd_ctrl.data[7] = joy->axes[vrt_rot];
	cmd_ctrl.data[8] = joy->axes[hrz_pd];
	cmd_ctrl.data[9] = joy->axes[vrt_pd];
	if(joy->buttons[zoom_in] > 0){
		cmd_ctrl.data[10] = 1;
	}else if(joy->buttons[zoom_out] > 0){
		cmd_ctrl.data[10] = -1;
	}else{
		cmd_ctrl.data[10] = 0;
	}
	
	std::cout<<"[0]"<<(int)cmd_ctrl.data[0]<<" [1]"<<(int)cmd_ctrl.data[1]<<" [2]"<<(int)cmd_ctrl.data[2]
		 <<" [3]"<<(int)cmd_ctrl.data[3]<<" [4]"<<(int)cmd_ctrl.data[4]<<" [5]"<<(int)cmd_ctrl.data[5]<<std::endl;
	std::printf("%1.6f %1.6f %1.6f %1.6f\n",cmd_ctrl.data[6],cmd_ctrl.data[7],cmd_ctrl.data[8],cmd_ctrl.data[9]);
	//printf("[0] %d\n",cmd_ctrl.data[0]);
	cmd_pub_.publish(cmd_ctrl);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "swing_core");
	TeleopJoy teleop_joy;

	ros::spin();
	return 0;
}
