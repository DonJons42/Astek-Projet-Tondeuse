#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <vesc_msgs/VescStateStamped.h>
#include <std_msgs/Float64.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

#define LENGTH_ANALYSIS 5000
#define MOTOR_CHECKING_MODE false  // true //false
using namespace message_filters;
using namespace geometry_msgs;

using namespace std;

//void callback(const vesc_msgs::VescStateStamped::ConstPtr& left_state,const vesc_msgs::VescStateStamped::ConstPtr& right_state);

class VESC_Diff_Drive
{
    public :
      //ros::NodeHandle nh;
	  std::string odom_frame_, base_frame_ ;
	  bool publish_tf_;
	  double x_, y_, yaw_; 
	  double speed_to_erpm_gain_, speed_to_erpm_offset_;
	  double wheel_base_;  // distance between the two wheels
	  double wheel_radius_;
      vesc_msgs::VescStateStamped::ConstPtr last_left_state_,last_right_state_;
	  ros::Publisher odom_pub_;
	  boost::shared_ptr<tf::TransformBroadcaster> tf_pub_;

	  ///////////////////////////////////////
      // Debbug : motor checking 
      ///////////////////////////////////////
      double tab_left_speed_[LENGTH_ANALYSIS];
      double tab_left_time_[LENGTH_ANALYSIS];
      double tab_right_speed_[LENGTH_ANALYSIS];
      double tab_right_time_[LENGTH_ANALYSIS];
      int indice_;
      bool motor_checking_mode_;
      double original_time_left_;
	  double original_time_right_;
      double left_to_right_;
      double right_to_left_;
      int length_analysis_;
      bool is_first_time_;

   private :   
      message_filters::Subscriber<vesc_msgs::VescStateStamped> left_motor_sub_;
      message_filters::Subscriber<vesc_msgs::VescStateStamped> right_motor_sub_;
      typedef sync_policies::ApproximateTime<vesc_msgs::VescStateStamped, vesc_msgs::VescStateStamped> MySyncPolicy;  
	  typedef Synchronizer<MySyncPolicy> Sync;
      boost::shared_ptr<Sync> sync;
      

   
    ///////////////////////////////////////////////////////////////////////
    //////
    //////////////////////////////////////////////////////////////////////
    public:
      VESC_Diff_Drive(ros::NodeHandle nh);
	  void callback(const vesc_msgs::VescStateStamped::ConstPtr& left_state,const vesc_msgs::VescStateStamped::ConstPtr& right_state);

};


///////////////////////////////////////////////////////////////////////
//////
//////////////////////////////////////////////////////////////////////
VESC_Diff_Drive::VESC_Diff_Drive(ros::NodeHandle nh)
{

      ///////////////////////////////////////
      // Debbug : motor checking 
      ///////////////////////////////////////
	   indice_=0;
       length_analysis_=LENGTH_ANALYSIS;
       motor_checking_mode_=MOTOR_CHECKING_MODE;
       is_first_time_=true;
  

		if (!nh.getParam("/speed_to_erpm_gain", speed_to_erpm_gain_))
				return ;

		if (!nh.getParam("/speed_to_erpm_offset", speed_to_erpm_offset_))
				return ;

		if (!nh.getParam("/wheelbase", wheel_base_))
				return ;

		if (!nh.getParam("/wheelradius", wheel_radius_))
				return ;

	  left_motor_sub_.subscribe(nh, "/left_wheel/sensors/core", 100);
	  right_motor_sub_.subscribe(nh, "/right_wheel/sensors/core",100);
	  ROS_INFO_STREAM("wheel_radius:" << wheel_radius_ );
	  ROS_INFO_STREAM("wheel_base:" << wheel_base_ );

 	  // ApproximateTimes takes a queue size N as its constructor arguments hence Sync(MySyncPolicy(N)
      sync.reset(new Sync(MySyncPolicy(2),left_motor_sub_, right_motor_sub_));
	  sync->registerCallback(boost::bind(&VESC_Diff_Drive::callback,this, _1, _2));



	  odom_frame_="odom";
	  base_frame_="base_link";
	  publish_tf_=false;
	  x_=0.0;
	  y_=0.0;
	  yaw_=0.0;

	  nh.param("odom_frame", odom_frame_, odom_frame_);
	  nh.param("base_frame", base_frame_, base_frame_);
		  
		 
	  nh.param("publish_tf", publish_tf_, publish_tf_);

		  // create odom publisher
	  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
		  // create tf broadcaster
	  if (publish_tf_) 
		  {
            ROS_INFO_STREAM("Odom tf published" );    
			tf_pub_.reset(new tf::TransformBroadcaster);
		  }
    
    
}

///////////////////////////////////////////////////////////////////////
//////
/////////////////////////////////////////////////////////////////////
void VESC_Diff_Drive::callback(const vesc_msgs::VescStateStamped::ConstPtr& left_state,const vesc_msgs::VescStateStamped::ConstPtr& right_state)
{

		  // convert to engineering units
		  //double left_current_speed = ( left_state->state.speed - speed_to_erpm_offset_ ) / speed_to_erpm_gain_;
          //ROS_INFO_STREAM("left_current_speed"<< left_current_speed );
		  double left_current_speed = ((2.0*3.14* left_state->state.speed)/(7.0*60.0))/6.0;
		  ROS_INFO_STREAM("left:" << left_current_speed );
		  // convert to engineering units
		  //double right_current_speed = ( right_state->state.speed - speed_to_erpm_offset_ ) / speed_to_erpm_gain_;
          //ROS_INFO_STREAM("right_current_speed"<< right_current_speed );
          double right_current_speed = (-(2.0*3.14* right_state->state.speed)/(7.0*60.0))/6.0;
          ROS_INFO_STREAM("right:" << right_current_speed );
         
		  
		  // use current state as last state if this is our first time here
		  if ( is_first_time_ )
		  {
            last_left_state_ = left_state;
            last_right_state_ = right_state;
            is_first_time_=false;
          }

         

		  // calc elapsed time
		  ros::Duration dt_left = (left_state->header.stamp - last_left_state_->header.stamp);
		  ros::Duration dt_right = (right_state->header.stamp - last_right_state_->header.stamp);



          ///////////////////////////////////////
      	  // Debbug : motor checking 
          ///////////////////////////////////////
          if( motor_checking_mode_ )
          {
              if(indice_==0)
              {
               //Register first time to substract 
                 original_time_left_=last_left_state_->header.stamp.toSec();
                 original_time_right_=last_right_state_->header.stamp.toSec();
                 left_to_right_ = original_time_right_ -  original_time_left_;
                 right_to_left_ = original_time_left_ -  original_time_right_;
                 if( left_to_right_ >= 0)
                 {
                    right_to_left_=0;
                 }else{
                       left_to_right_=0;
                       }
 
                  
              }
              
		      if(indice_<length_analysis_)
			  {
		        tab_left_speed_[indice_]=left_current_speed;
		        tab_left_time_[indice_]=(left_state->header.stamp).toSec()- original_time_left_ + right_to_left_;
		        tab_right_speed_[indice_]=right_current_speed;
		        tab_right_time_[indice_]=(right_state->header.stamp).toSec()- original_time_right_ + left_to_right_ ;
		        indice_=indice_+1;
                ROS_INFO_STREAM("etape : " << indice_ );

		      }else{
				        if(indice_==length_analysis_)
				        {
						     
						     ofstream fichier("/home/lab-robot/Bureau/astek_ws/src/vesc/vesc_diff_drive/Motor_data/Motor_test.txt",ios::out | ios::trunc );
						     if(fichier)
						     {
								  fichier<<"left speed in rad/s:"<<endl;
								  for(int i=0;i<length_analysis_;i++)
								  {
								   fichier<<tab_left_speed_[i]<<",";
								   if (i==length_analysis_-1){
								   fichier<<tab_left_speed_[i];}
								  }
								
                                  fichier<< endl;  
								  fichier<<"left time in s:"<<endl;
								  for(int i=0;i<length_analysis_;i++)
								  {
								   fichier<< tab_left_time_[i]<<",";
								   if (i==length_analysis_-1){
								   fichier<<tab_left_time_[i];}
								  }
                                  fichier<< endl;
								  fichier<<"right speed in rad/s:"<<endl;
								  for(int i=0;i<length_analysis_;i++)
								  {
								   fichier<<tab_right_speed_[i]<<",";
								   if (i==length_analysis_-1){
								   fichier<<tab_right_speed_[i];}
								  }
                                  fichier<< endl;  
								  fichier<<"right time in s:"<<endl;
								  for(int i=0;i<length_analysis_;i++)
								  {
								   fichier<< tab_right_time_[i]<<",";
								   if (i==length_analysis_-1){
								   fichier<<tab_right_time_[i];}
								  }
                                  fichier<< endl;
								  fichier.close();
								  indice_=indice_+1;
						    }else{
						         std::cout<<"Cannot open file "<<std::endl;
						         }
						}
		           }
            }



		  double delta_t=(dt_left.toSec()+dt_right.toSec())/2.0;

		  // propigate odometry
		  double v = (1.0/2.0)*wheel_radius_*(left_current_speed +right_current_speed );
		  double w = (1.0/ wheel_base_ )*wheel_radius_*(right_current_speed-left_current_speed );

		  // Compute the Delta_x Delta_y, Delta_theta
		  double delta_x;
          double delta_y;
          double delta_yaw;
		 // If omega is not zero ( so no division with zero )
         if(w!=0)
         {
                 delta_x =(v/w)*(sin(yaw_+delta_t*w)-sin(yaw_));
                 delta_y=-(v/w)*(cos(yaw_+delta_t*w)-cos(yaw_));
		         delta_yaw=w*delta_t;
         }else{
                 delta_x=delta_t*v*cos(yaw_);
                 delta_y=delta_t*v*sin(yaw_);
		         delta_yaw=0;
              }

        

		  x_ += delta_x;
          //ROS_INFO_STREAM("delta_x" << delta_x );
		  y_ += delta_y;
		  yaw_ += delta_yaw;

		  // save state for next time
		  last_left_state_= left_state;
		  last_right_state_ = right_state;
          //ROS_INFO_STREAM(" last left state" << last_left_state_->state.speed );

		  // publish odometry message
		  nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
		  odom->header.frame_id = odom_frame_;
		  odom->header.stamp = left_state->header.stamp;
		  odom->child_frame_id = base_frame_;

		  // Position
		  odom->pose.pose.position.x = x_;
          //ROS_INFO_STREAM(" x_" << x_ );
		  //ROS_INFO_STREAM(" odom->pose.pose.position.x" << odom->pose.pose.position.x );
		  odom->pose.pose.position.y = y_;
		  odom->pose.pose.orientation.x = 0.0;
		  odom->pose.pose.orientation.y = 0.0;
		  odom->pose.pose.orientation.z = sin(yaw_/2.0);
		  odom->pose.pose.orientation.w = cos(yaw_/2.0);

		  // Position uncertainty
		  // @todo Think about position uncertainty, perhaps get from parameters? 
		  odom->pose.covariance[0]  = 0.2; ///< x
		  odom->pose.covariance[7]  = 0.2; ///< y
		  odom->pose.covariance[35] = 0.4; ///< yaw

		  // Velocity ("in the coordinate frame given by the child_frame_id")
		  odom->twist.twist.linear.x = v;
		  odom->twist.twist.linear.y = 0.0;
		  odom->twist.twist.angular.z =w;

		  // Velocity uncertainty
		  // @todo Think about velocity uncertainty //

		  if (publish_tf_) 
			{
				geometry_msgs::TransformStamped tf;
				tf.header.frame_id = odom_frame_;
				tf.child_frame_id = base_frame_;
				tf.header.stamp = ros::Time::now();
				tf.transform.translation.x = x_;
				tf.transform.translation.y = y_;
				tf.transform.translation.z = 0.0;
				tf.transform.rotation = odom->pose.pose.orientation;
				if (ros::ok()) 
				{
				  tf_pub_->sendTransform(tf);
				}
		    }

		  if (ros::ok()) 
		  {
			odom_pub_.publish(odom);
		  }
		

		//Update the last state 
		last_left_state_ = left_state;
		last_right_state_ = right_state;

	}



////////////////////////////////////////////////////////////////////////////////
// MAIN 
////////////////////////////////////////////////////////////////////////////////

	int main(int argc, char** argv)
	{
	  ros::init(argc, argv, "odom_estimate");
      ros::NodeHandle nh;
	 /* std::string odom_frame_, base_frame_ ;
	  bool publish_tf_;
	  double x_, y_, yaw_; 
	  double speed_to_erpm_gain_, speed_to_erpm_offset_;
	  double wheel_base_;  // distance between the two wheels
	  double wheel_radius_;	*/

      VESC_Diff_Drive VESC_Diff_Drive_instance(nh);
	  ros::spin();

	  return 0;
	}



