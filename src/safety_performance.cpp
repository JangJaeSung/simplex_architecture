//Safety Performance Controller
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

using namespace std;

class Safety_Performance{
protected:
	ros::NodeHandle nh_;

	ros::Subscriber vehicle_info_of_hpcp_sub_;
	ros::Subscriber mode_check_sub_;

	float steer_;
	float throttle_;
public:
	Safety_Performance();
	~Safety_Performance();
	void StateCallback(const std_msgs::Bool::ConstPtr& state); 
	void VehicleCallback(const geometry_msgs::Twist::ConstPtr& data);
	void Decision_Logic();
};

Safety_Performance::Safety_Performance(){
	//vehicle_info_of_hpcp_sub_ = nh_.subscribe<geometry_msgs::Twist>("vehicle_info_of_hpcp", 100, &Safety_Performance::VehicleCallback, this);
	mode_check_sub_ = nh_.subscribe<std_msgs::Bool>("mode", 100, &Safety_Performance::StateCallback, this);


}

Safety_Performance::~Safety_Performance(){


}

void Safety_Performance::StateCallback(const std_msgs::Bool::ConstPtr& state){
	if(state->data == 1){
		ROS_INFO("start hpcp");
		vehicle_info_of_hpcp_sub_ = nh_.subscribe<geometry_msgs::Twist>("vehicle_info_of_hpcp", 100, &Safety_Performance::VehicleCallback, this);
		//ros::spinOnce();
	}
	else if(state->data == 0){
		ROS_INFO("Start Safety Performance Mode");
		Decision_Logic();
	}

}

void Safety_Performance::VehicleCallback(const geometry_msgs::Twist::ConstPtr& data){
	steer_ = data->angular.z;
	throttle_ = data->linear.x;
	ROS_INFO("hpcp callback!!");

}

void Safety_Performance::Decision_Logic(){
	ROS_INFO("Decision Logic Execution");
}

int main(int argc, char **argv){
	ros::init(argc, argv, "Safety_Performance_Mode");

	Safety_Performance Safety_Performance;

	while(ros::ok()){
		ros::spinOnce();

	}


	return 0;
}
