#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sstream>


class Cmd_vel_control
{
    public :
      std_msgs::Float64 wr_;
      std_msgs::Float64 wl_;
      double L;
      double r;
      ros::Publisher left_pub ;
      ros::Publisher right_pub ;
      ros::Subscriber cmd_sub;



      Cmd_vel_control(ros::NodeHandle nh);
      void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& speedl);

};


Cmd_vel_control::Cmd_vel_control(ros::NodeHandle nh)
{
  left_pub = nh.advertise<std_msgs::Float64>("/left_wheel/commands/motor/speed", 5);
  right_pub = nh.advertise<std_msgs::Float64>("/right_wheel/commands/motor/speed", 5);
    L = 0.525; /// wheelbase
    r = 0.1; /// rayon des roues
  cmd_sub = nh.subscribe("/cmd_vel_to_rpm",100, &Cmd_vel_control::cmd_vel_callback, this);
}

void Cmd_vel_control::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& speedl)
{
    double v;
    double omega;
    v = speedl->linear.x;
    omega = speedl->angular.z;
    double wr;
    double wl;
    wr = (((2*v+omega*L)/(2.0*r))*60.0*6.0*7.0)/(2*3.14);  /// [(speed(tr/s)*60)(tr/min)]*7*6=RPM*7*Courroie=ERPM*courroie;
    wl = (((2*v-omega*L)/(2.0*r))*60.0*6.0*7.0)/(2*3.14);  /// [(speed(tr/s)*60)(tr/min)]*7*6=RPM*7*courroie=ERPM*courroie;
  
    wr_.data = -wr;
    wl_.data = wl;
    right_pub.publish(wr_);
    left_pub.publish(wl_);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_to_erpm");
  ros::NodeHandle nh;   
  Cmd_vel_control cmd_vel_control(nh);
	ros::spin();
  return 0;
}
