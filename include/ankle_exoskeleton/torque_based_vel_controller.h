#ifndef DYNAMIXEL_CURRENT_BASED_PWM_CONTROLLER_H
#define DYNAMIXEL_TORQUE_BASED_VELOCITY_CONTROLLER_H


#include <ros/ros.h>
#include <ros/time.h>
#include <signal.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <ankle_exoskeleton/DynamixelStatusList.h>
#include <ankle_exoskeleton/DynamixelCmdSimplified.h>

class DynamixelTorqueBasedVelocityController
{
public:
  DynamixelTorqueBasedVelocityController();
  void run();
  void publishOnShutdown();

private:
  
  void desiredValueCallback(const std_msgs::Float32::ConstPtr& msg);
  void dynamixelStatusCallback(const ankle_exoskeleton::DynamixelStatusList::ConstPtr& msg);
  int32_t proportionalController(int32_t error);
  int32_t derivativeController(int32_t error);
  double lowPassFilter(double input);
  

  ros::NodeHandle nh_;
  ros::Subscriber goal_desired_value_sub_;
  ros::Subscriber dynamixel_status_sub_;
  ros::Publisher goal_velocity_pub_;
  
  std_msgs::Int32 velocity_msg_;
   
  std::vector<int16_t> present_current_vector_;
  std::vector<int16_t> present_velocity_vector_;
  double present_current_;
  double present_velocity_;
  double motor_id_;
  bool is_motorState_updated_;
  
  double desired_value_;
  bool is_desired_value_updated_;
  
  ros::Time previous_time_;
  double previous_filter_value_;

  
};

#endif  // DYNAMIXEL_TORQUE_BASED_VELOCITY_CONTROLLER_H