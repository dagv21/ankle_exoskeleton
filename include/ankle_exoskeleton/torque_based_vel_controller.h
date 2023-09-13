#ifndef DYNAMIXEL_TORQUE_BASED_VELOCITY_CONTROLLER_H
#define DYNAMIXEL_TORQUE_BASED_VELOCITY_CONTROLLER_H


#include <ros/ros.h>
#include <ros/time.h>
#include <signal.h>

#include <yaml-cpp/yaml.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <ankle_exoskeleton/DynamixelStatusList.h>
#include <ankle_exoskeleton/DynamixelCmdSimplified.h>

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

class DxlTorqueBasedVelController
{

private:
    // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher goal_velocity_pub_;
  
  // ROS Topic Subscriber
  ros::Subscriber goal_desired_value_sub_;
  ros::Subscriber dynamixel_status_sub_;

  std::map<std::string, uint32_t> dynamixel_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  

  std::vector<int16_t> present_current_vector_;
  std::vector<int16_t> present_velocity_vector_;
  
  double present_current_;
  double present_velocity_;
  double desired_value_;
  
  bool is_motorState_updated_;
  bool is_desired_value_updated_;

  ros::Time previous_control_time_;

  // Butterworth Filter coefficients and variables
  double a0, a1, a2, b1, b2;
  double x1, x2, y1, y2;
  
  ros::Time previous_filter_time_;
  double previous_filter_value_;

  std_msgs::Int32 velocity_msg_;

public:
  DxlTorqueBasedVelController();
  ~DxlTorqueBasedVelController();

  bool getDynamixelsInfo(const std::string yaml_file);

  void initPublisher(void);
  void initSubscriber();

  void setMotorTorque(int value);

  void desiredValueCallback(const std_msgs::Float32::ConstPtr& msg);
  void dynamixelStatusCallback(const ankle_exoskeleton::DynamixelStatusList::ConstPtr& msg);
  
  void butterworthInitialization();
  double lowPassButterworthFilter(double input);
  double lowPassFilter(double input);

  void run();

  std::string namespace_str_;
  double motor_id_;

};

#endif  // DYNAMIXEL_TORQUE_BASED_VELOCITY_CONTROLLER_H
