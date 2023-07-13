/* Original Authors: Daniel Gomez-Vargas */

#include "../../include/ankle_exoskeleton/current_based_pwm_controller.h"

DynamixelCurrentBasedPWMController::DynamixelCurrentBasedPWMController()
{
  goal_desired_value_sub_ = nh_.subscribe("/ankle_exo_frontal/motor/desired_value", 1, &DynamixelCurrentBasedPWMController::desiredValueCallback, this);
  dynamixel_status_sub_ = nh_.subscribe("/ankle_exo_frontal/dynamixel_status", 1, &DynamixelCurrentBasedPWMController::dynamixelStatusCallback, this);
  goal_velocity_pub_ = nh_.advertise<std_msgs::Int32>("/ankle_exo_frontal/motor/goal_velocity", 1);
  goal_current_pub_ = nh_.advertise<std_msgs::Int32>("/ankle_exo_frontal/motor/goal_current", 1);
  // Set the control frequency to 200 Hz
  is_desired_value_updated_ = false;
  is_motorState_updated_ = false;
  motor_id_ = 1;
  previous_time_ = ros::Time::now();
  previous_error_ = 0.0;
  previous_filter_value_ = 0.0;
}

void DynamixelCurrentBasedPWMController::desiredValueCallback(const std_msgs::Float32::ConstPtr& msg)
{
  desired_value_ = msg->data;
  is_desired_value_updated_ = true;
}

void DynamixelCurrentBasedPWMController::dynamixelStatusCallback(const ankle_exoskeleton::DynamixelStatusList::ConstPtr& msg)
{
  for (const auto& dynamixel_status : msg->dynamixel_status)
  {
    if (dynamixel_status.id == motor_id_)
    {
      if (present_current_vector_.size() < 5)
      {
        present_current_vector_.push_back(dynamixel_status.present_current);
        present_velocity_vector_.push_back(dynamixel_status.present_velocity);
        is_motorState_updated_ = false;
      }
      else{
        double sum_current = 0.0;
        double sum_velocity = 0.0;
        int size = present_current_vector_.size();

        for (int i = 0; i < size; ++i) {
          sum_current += static_cast<double>(present_current_vector_[i]);
          sum_velocity += static_cast<double>(present_velocity_vector_[i]);
        }
        present_current_ = (sum_current / size)*0.0036; // Register value to [A]
        present_velocity_ = (sum_velocity / size)*0.229; //Register value to [rpm]
        is_motorState_updated_ = true;

        present_current_vector_.erase(present_current_vector_.begin());
        present_velocity_vector_.erase(present_velocity_vector_.begin());
      }
    }
  }
}

int32_t DynamixelCurrentBasedPWMController::proportionalController(int32_t error)
{
  double kp = 1;
  double vel = error;
  int32_t control_value = -kp * desired_value_ * vel;
  is_motorState_updated_ = false;
  std::cout << control_value << '\n';
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

void DynamixelCurrentBasedPWMController::publishOnShutdown() //TODO
{
  std_msgs::Int32 pwm_msg;
  pwm_msg.data = 0; // Set the PWM control value to 0
  goal_velocity_pub_.publish(pwm_msg);
}

double DynamixelCurrentBasedPWMController::lowPassFilter(double input)
{
  double filterFactor = 0.9;
  double output = filterFactor * input + (1.0 - filterFactor) * previous_filter_value_;
  previous_filter_value_ = output; // Update previous value for the next iteration
  return output;
}

void DynamixelCurrentBasedPWMController::run()
{
  // // Create a step signal from 0 to 55 RPM
  // double target_rpm = 0.0;
  // double step_size = 0.8; // Adjust the step size as needed

  ros::Rate rate(300);
  ROS_INFO("Starting Controller");
  while (ros::ok())
  {

    if (is_desired_value_updated_ && is_motorState_updated_)
    {
      // Torque Control based on Current Feedback (Low Level: Velocity Control)
      double Bv_gain = 0.015;

      double K_motor = 1.8;  // Nm/A
      double desired_current = (desired_value_ / K_motor); // Torque to Current

      double error_current = desired_current - present_current_;
      if (abs(error_current) <= 0.1) // Tolerance of 50 mA
        error_current = 0;

      double u_velocity = error_current * 1/Bv_gain ;

      if (abs(u_velocity) < 2) // Tolerance of 2 rpm
        u_velocity = 0;

      if (u_velocity >= 55) // Motor Saturation
        u_velocity = 55;
      else if (u_velocity <= -55)
        u_velocity = -55;

      double u_velocity_filtered = lowPassFilter(u_velocity);

      int32_t velocity_control = u_velocity_filtered/0.229;
      std::cout << "Error: " << error_current << "\tDesired Motor Velocity: " << u_velocity << "rpm" << "\tRegister Velocity: " << velocity_control <<'\n';

      std_msgs::Int32 velocity_msg;
      velocity_msg.data = velocity_control;
      goal_velocity_pub_.publish(velocity_msg);

      // Velocity Control based on Velocity Feedback (Low Level: Current Control)
      double Bc_gain = 0.02;

      double u_current = (desired_value_ + present_velocity_) * Bc_gain;

      // std::cout << "Desired Current: " << desired_current << "\tVelocity: " << present_velocity_ << "\tU_cur: " << u_current << '\n';

      if (abs(u_current) < 0.01) // Tolerance of 10 mA
        u_current = 0;

      if (abs(u_current) >= 6.0) // Motor Saturation
        u_current = 6;

      int32_t current_control = u_current/0.0036;
      // std::cout << "Desired Motor Current: " << u_current << "A" << "\tRegister Current: " << current_control <<'\n';

      std_msgs::Int32 current_msg;
      current_msg.data = current_control;
      goal_current_pub_.publish(current_msg);

      is_motorState_updated_ = false;


      // // Increment the target RPM
      // // target_rpm += step_size;
      //
      // // Check if the target RPM exceeds 55
      // if (present_velocity_ >= 52.0)
      // {
      //
      //   target_rpm = -55.0;
      //
      //   // Create a message to publish the desired RPM
      //   int32_t velocity_control = target_rpm/0.229;
      //   std_msgs::Int32 rpm_msg;
      //   rpm_msg.data = velocity_control;
      //
      //   // Publish the message
      //   goal_velocity_pub_.publish(rpm_msg);
      //
      //   std::this_thread::sleep_for(std::chrono::seconds(1));
      //   // step_size = -step_size;
      // }
      // if (present_velocity_ <= -52.0)
      // {
      //
      //   target_rpm = 55.0;
      //
      //   // Create a message to publish the desired RPM
      //   int32_t velocity_control = target_rpm/0.229;
      //   std_msgs::Int32 rpm_msg;
      //   rpm_msg.data = velocity_control;
      //
      //   // Publish the message
      //   goal_velocity_pub_.publish(rpm_msg);
      //
      //   std::this_thread::sleep_for(std::chrono::seconds(1));
      //   // step_size = -step_size;
      // }


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
