// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_TO_DIFF_DRIVE_H_
#define VESC_TO_DIFF_DRIVE_H_

#include <ros/ros.h>
#include <vesc_msgs/VescStateStamped.h>
#include <std_msgs/Float64.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace vesc_to_diff_drive
{
using namespace message_filters;
typedef      message_filters::sync_policies::ApproximateTime<vesc_msgs::VescStateStamped, vesc_msgs::VescStateStamped> MySyncPolicy;

	  
	class vesc_to_diff_drive
	{
	public:

	vesc_to_diff_drive(ros::NodeHandle nh, ros::NodeHandle private_nh);
    Synchronizer<MySyncPolicy> sync;
	private:
	  // ROS parameters
	  std::string odom_frame_;
	  std::string base_frame_;

	  // conversion gain and offset
	  double speed_to_erpm_gain_, speed_to_erpm_offset_;
	  double wheel_base_;  // distance between the two wheels
	  double wheel_radius_;	  
	  bool publish_tf_;

	  // odometry state
	  double x_, y_, yaw_;
	  vesc_msgs::VescStateStamped::ConstPtr last_left_state_; ///< Last received state message
	  vesc_msgs::VescStateStamped::ConstPtr last_right_state_; ///< Last received state message

	  //Listen synchronously the the right and left topics 
	  message_filters::Subscriber<vesc_msgs::VescStateStamped> left_motor_sub_; 
	  message_filters::Subscriber<vesc_msgs::VescStateStamped> right_motor_sub_;


          // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	  //If some messages are of a type that doesn't contain the header field,  ApproximateTimeSynchronizer refuses by default adding such messages        

  
	// ROS services
	  ros::Publisher odom_pub_;
	  boost::shared_ptr<tf::TransformBroadcaster> tf_pub_;

	  // ROS callbacks
	  void vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& left_state,
		                 const vesc_msgs::VescStateStamped::ConstPtr& right_state);
	};

} // namespace  vesc_to_diff_drive

#endif // VESC_TO_DIFF_DRIVE_H_
