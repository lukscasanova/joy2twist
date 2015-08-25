/**
 * @file   joy2twist.cpp
 * @author Lucas Casanova Nogueira <lukscasanova@gmail.com>
 * @date   08/25/2015
 * @brief  ROS Node for getting Twist messages out of a PS3 controller
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

using namespace std;

class Joy2Twist
{
public:
  Joy2Twist();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void UpdateCmdVelFromJoystick (double raw_linear, double raw_angle);
  void UpdateStamped();
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::TwistStamped cmd_vel_stamped_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber joy_sub_;
  bool stamped_;
  string frame;
};

/**
 *	@brief fills an Twist message from a linear(x) and angular(z) velocity
 */

void Joy2Twist::UpdateCmdVelFromJoystick (double raw_linear, double raw_angle){
	cmd_vel.linear.x=raw_linear;
	cmd_vel.linear.y=0.0;
	cmd_vel.linear.z=0.0;
	cmd_vel.angular.x=0.0; 
	cmd_vel.angular.y=0.0;
	cmd_vel.angular.z=raw_angle;
} 

/**
 * @brief fills an TwistStamped message using the current stored Twist and the current time
 */
void Joy2Twist::UpdateStamped(){
	cmd_vel_stamped_.twist = cmd_vel;
	cmd_vel_stamped_.header.stamp = ros::Time::now();
}

Joy2Twist::Joy2Twist():
  linear_(1),
  angular_(2),
  stamped_(false),
  frame(""),
  nh_("~")
{

  //These parameters indicate which fields from the joy message contains
  // the desired linear and angular axis, and a scale factor to be used.
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("stamped", stamped_, stamped_);
  nh_.param("twist_frame", frame, frame);
 
 if(stamped_){
 	cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);	
 }else{
 	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);	
 }
 
 joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy2Twist::joyCallback, this);
 cmd_vel_stamped_.header.frame_id=frame;
}

void Joy2Twist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO_STREAM("\nLinear: " << joy->axes[linear_] << "\nAngular: " << joy->axes[angular_]);
  UpdateCmdVelFromJoystick(l_scale_*joy->axes[linear_],a_scale_*joy->axes[angular_]);
  if(!stamped_){
  	cmd_vel_pub_.publish(cmd_vel);
  }else{
  	UpdateStamped();
  	cmd_vel_pub_.publish(cmd_vel_stamped_);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy2twist");
  Joy2Twist joy2twist;

  ros::spin();
}
