//Mode Checker
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

#include <time.h>

using namespace std;
clock_t previous_time_ = 0;
clock_t current_time_ = 0;
clock_t pre_callback_time_;
clock_t callback_time_;
int callback_count = 0;

class checker{
protected:
	ros::NodeHandle nh_;

	ros::Subscriber vehicle_info_sub_;
	ros::Publisher vehicle_info_of_hpcp_pub_;
	ros::Publisher mode_check_pub_;
	
	geometry_msgs::Twist vehicle_info_of_hpcp_;
	std_msgs::Bool mode_check_;

	bool current_state_ = true;
	bool safety_state_= false;

public:
	checker();
	~checker();
	void VehicleCallback(const geometry_msgs::Twist::ConstPtr& data);
	void Publish_in_safety_mode();
};

checker::checker(){
	ROS_INFO("Check! High Performance Mode");
	vehicle_info_sub_ = nh_.subscribe<geometry_msgs::Twist>("vehicle_info", 100, &checker::VehicleCallback, this);

	vehicle_info_of_hpcp_pub_ = nh_.advertise<geometry_msgs::Twist>("vehicle_info_of_hpcp", 100);
	mode_check_pub_ = nh_.advertise<std_msgs::Bool>("mode", 100);
}

checker::~checker(){

}

void checker::VehicleCallback(const geometry_msgs::Twist::ConstPtr& data){
	previous_time_ = clock();

	vehicle_info_of_hpcp_.linear.x = data->linear.x;
	vehicle_info_of_hpcp_.angular.x = data->angular.z;
	mode_check_.data = current_state_;

	vehicle_info_of_hpcp_pub_.publish(vehicle_info_of_hpcp_);
	mode_check_pub_.publish(mode_check_);

	current_time_ = clock();
}

void checker::Publish_in_safety_mode(){
	mode_check_.data = safety_state_;
	mode_check_pub_.publish(mode_check_);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "Checker");

	checker checker;

	while(ros::ok()){
		pre_callback_time_ = current_time_ - previous_time_;
		ros::spinOnce();

		ros::Duration(0.1).sleep();

		callback_time_ = current_time_ - previous_time_;

		//check if the callback has stopped or not
		if(callback_time_ != 0 && (callback_time_ - pre_callback_time_) == 0){
			callback_count++;
		}
		else{
			callback_count = 0 ;
		}

		//switch to safety mode
		if (callback_count >= 10){
			checker.Publish_in_safety_mode();	
		}
	}
	return 0;
}
