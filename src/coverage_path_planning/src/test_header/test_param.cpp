#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "file_include.cpp"

using namespace std;

int main(int argc, char *argv[]){
  srand(time(0));
	ros::init(argc, argv, "next_goal");
	ros::NodeHandle save_path;

	Nombre_test machin(2, 3.5);

	while(ros::ok()){
		fonction_include(machin);
	}
	return 0;
}
