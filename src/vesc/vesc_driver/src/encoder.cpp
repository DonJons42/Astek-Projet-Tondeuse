#include <vesc_msgs/VescStateStamped.h>
#include <std_msgs/Float32.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>

using namespace std;


void data_motor1(const vesc_msgs::VescStateStamped &data){

ofstream encod_info;
encod_info.open("encoder1.txt");
encod_info.seekp(0,ios_base::end);
encod_info << data.state.current_motor << "\n" 
	       << data.state.current_input << "\n"
		   << data.state.speed << "\n"
		   << data.state.duty_cycle << "\n"
		   << data.state.displacement << "\n"
		   << data.state.distance_traveled << "\n";

encod_info.close();
}


void data_motor2(const vesc_msgs::VescStateStamped &data){

ofstream encod_info;
encod_info.open("encoder2.txt");
encod_info << data.state.current_motor << "\n" 
	       << data.state.current_input << "\n"
		   << data.state.speed << "\n"
		   << data.state.duty_cycle << "\n"
		   << data.state.displacement << "\n"
		   << data.state.distance_traveled << "\n";

encod_info.close();
}

int main(int argc, char *argv[]){

ros::init(argc,argv,"encoder");
ros::NodeHandle nh;
ros::Subscriber motor1 = nh.subscribe("node1/sensors/core", 10,data_motor1);
ros::Subscriber motor2 = nh.subscribe("node2/sensors/core", 10,data_motor2);

while (ros::ok()){
	ros::spinOnce();
	}
return 0;
}


          
