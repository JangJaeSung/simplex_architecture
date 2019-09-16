//High Performance Controller
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

using namespace std;

class High_Performance{
protected:
	ros::NodeHandle nh_;
	ros::Publisher vehicle_info_pub_;
	ros::Subscriber performance_state_sub_;

	float steer_ = 1500.0;
	float throttle_ = 1500.0;

	geometry_msgs::Twist vehicle_info_;
public:
	High_Performance();
	~High_Performance();
	void StateCallback(const std_msgs::Bool::ConstPtr& state);


};

High_Performance::High_Performance(){
	vehicle_info_pub_ = nh_.advertise<geometry_msgs::Twist>("vehicle_info", 100);
	performance_state_sub_ = nh_.subscribe<std_msgs::Bool>("state", 100, &High_Performance::StateCallback, this);

	ROS_INFO("start class");
}

High_Performance::~High_Performance(){


}


void High_Performance::StateCallback(const std_msgs::Bool::ConstPtr& state){
	ROS_INFO("Start High Performance Mode");
	//ROS_INFO("STATE = %d ", state->data);

	ros::Rate r(10);

	if(state->data == 1){
		while(ros::ok()){
			vehicle_info_.linear.x = throttle_;	
			vehicle_info_.angular.z = steer_;	

			vehicle_info_pub_.publish(vehicle_info_);
			ros::spinOnce();
			r.sleep();
		}
	}
	else if(state->data == 0){
		ROS_INFO("High Performance Mode Failed!!!");
		ros::shutdown();
	}
}



int main(int argc, char **argv){
	ros::init(argc, argv, "High_Performance_Mode");

	High_Performance High_Performance;

	ros::spin();

	return 0;
}
