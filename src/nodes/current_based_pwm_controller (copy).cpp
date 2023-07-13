/* Original Authors: Daniel Gomez-Vargas */

#include "../../include/ankle_exoskeleton/current_based_pwm_controller.h"

DynamixelCurrentBasedPWMController::DynamixelCurrentBasedPWMController()
{
  goal_current_sub_ = nh_.subscribe("/ankle_exo_frontal/motor/goal_current", 1, &DynamixelCurrentBasedPWMController::goalCurrentCallback, this);
  dynamixel_status_sub_ = nh_.subscribe("/ankle_exo_frontal/dynamixel_status", 1, &DynamixelCurrentBasedPWMController::dynamixelStatusCallback, this);
  goal_pwm_pub_ = nh_.advertise<std_msgs::Int32>("/ankle_exo_frontal/motor/goal_pwm", 1);
  // Set the control frequency to 200 Hz
  is_current_updated_ = false;
  is_motorState_updated_ = false;
  motor_id_ = 1;
  previous_time_ = ros::Time::now();
  previous_error_ = 0.0;
}

void DynamixelCurrentBasedPWMController::goalCurrentCallback(const std_msgs::Int32::ConstPtr& msg)
{
  desired_current_ = msg->data;
  is_current_updated_ = true;
}

void DynamixelCurrentBasedPWMController::dynamixelStatusCallback(const ankle_exoskeleton::DynamixelStatusList::ConstPtr& msg)
{
  for (const auto& dynamixel_status : msg->dynamixel_status)
  {
    if (dynamixel_status.id == motor_id_)
    {
      if (present_current_value_.size() < 10)
      {
        present_current_value_.push_back(dynamixel_status.present_current);
        is_motorState_updated_ = false;
      }
      else{
        double sum = 0.0;
        for (const auto& value : present_current_value_) {
          sum += static_cast<double>(value);
        }
        present_current_ = sum / present_current_value_.size();
        is_motorState_updated_ = true;
        present_current_value_.erase(present_current_value_.begin());
      }
    }
  }
}

int32_t DynamixelCurrentBasedPWMController::proportionalController(int32_t error)
{
  double kp = 2;
  int32_t control_value = kp * error;
  is_motorState_updated_ = false;
  return control_value;
}

int32_t DynamixelCurrentBasedPWMController::derivativeController(int32_t error)
{
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - previous_time_).toSec();

  double kd = 0.2;
  int32_t control_value = static_cast<int32_t>(kd * (error - previous_error_)/dt);

  previous_error_ = error;
  previous_time_ = current_time;

  is_motorState_updated_ = false;

  return control_value;
}

void DynamixelCurrentBasedPWMController::publishOnShutdown()
{
  std_msgs::Int32 pwm_msg;
  pwm_msg.data = 0; // Set the PWM control value to 0
  goal_pwm_pub_.publish(pwm_msg);
}

void DynamixelCurrentBasedPWMController::run()
{
  ros::Rate rate(300);
  ROS_INFO("Starting Current Based PWM Controller");
  while (ros::ok())
  {
    if (is_current_updated_ && is_motorState_updated_)
    {
      int32_t error = desired_current_ - present_current_;
      if (abs(error) <= 10) //Tolerance of 33mA
      error = 0;
      int32_t pwm_control = - proportionalController(error) - derivativeController(error);
      std_msgs::Int32 pwm_msg;
      pwm_msg.data = pwm_control;
      goal_pwm_pub_.publish(pwm_msg);

    }
    ros::spinOnce();  // Handle callbacks
    rate.sleep();  // Maintain the desired loop rate
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_current_based_pwm_controller");
  DynamixelCurrentBasedPWMController controller;

  // ros::NodeHandle nh;
  // ros::Subscriber shutdown_sub = nh.subscribe("/rosout", 1, &DynamixelCurrentBasedPWMController::publishOnShutdown, &controller);

  controller.run();

  return 0;
}
