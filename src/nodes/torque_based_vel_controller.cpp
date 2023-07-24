/* Original Authors: Daniel Gomez-Vargas */

#include "../../include/ankle_exoskeleton/torque_based_vel_controller.h"

DynamixelTorqueBasedVelocityController::DynamixelTorqueBasedVelocityController()
{
  goal_desired_value_sub_ = nh_.subscribe("/ankle_exo_frontal/motor/desired_value", 1, &DynamixelTorqueBasedVelocityController::desiredValueCallback, this);
  dynamixel_status_sub_ = nh_.subscribe("/ankle_exo_frontal/dynamixel_status", 1, &DynamixelTorqueBasedVelocityController::dynamixelStatusCallback, this);
  goal_velocity_pub_ = nh_.advertise<std_msgs::Int32>("/ankle_exo_frontal/motor/goal_velocity", 1);

  ros::ServiceClient client = nh_.serviceClient<ankle_exoskeleton::DynamixelCmdSimplified>("/ankle_exo_frontal/torque_enable");
  ankle_exoskeleton::DynamixelCmdSimplified srv;
  srv.request.id = 1;
  srv.request.value = 1;
  if (client.call(srv)) {
      ROS_INFO("Torque enabled in Motor ID : %d",srv.request.id);
  } else {
      ROS_ERROR("Failed to enable torque of Motor ID : %d",srv.request.id);
  }

  is_desired_value_updated_ = false;
  is_motorState_updated_ = false;
  motor_id_ = 1;

  desired_value_ = 0.0; // Initial desired value 0.0 Nm

  previous_filter_value_ = 0.0;
}

void DynamixelTorqueBasedVelocityController::desiredValueCallback(const std_msgs::Float32::ConstPtr& msg)
{
  desired_value_ = msg->data;
  // is_desired_value_updated_ = true;
}

void DynamixelTorqueBasedVelocityController::dynamixelStatusCallback(const ankle_exoskeleton::DynamixelStatusList::ConstPtr& msg)
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

double DynamixelTorqueBasedVelocityController::lowPassFilter(double input)
{
  double filterFactor = 0.9;
  double output = filterFactor * input + (1.0 - filterFactor) * previous_filter_value_;
  previous_filter_value_ = output; // Update previous value for the next iteration
  return output;
}

void DynamixelTorqueBasedVelocityController::run()
{
  ros::Rate rate(300);
  ROS_INFO("Starting Controller");
  while (ros::ok())
  {
    if (is_motorState_updated_)
    {
      // Torque Control based on Current Feedback (Low Level: Velocity Control)
      double Bv_gain = 0.015;

      double K_motor = 1.8;  // Nm/A
      double desired_current = (desired_value_ / K_motor); // Torque to Current

      double error_current = desired_current - present_current_;
      if (abs(error_current) <= 0.09) // Tolerance of 50 mA
        error_current = 0;

      double u_velocity = error_current * 1/Bv_gain ;

      if (abs(u_velocity) < 3) // Tolerance of 2 rpm
        u_velocity = 0;

      if (u_velocity >= 55) // Motor Saturation in 55 rpm
        u_velocity = 55;
      else if (u_velocity <= -55)
        u_velocity = -55;

      double u_velocity_filtered = lowPassFilter(u_velocity);

      int32_t velocity_control = u_velocity_filtered/0.229;
      std::cout << "Error: " << error_current << "\tDesired Motor Velocity: " << u_velocity << "rpm" << "\tRegister Velocity: " << velocity_control <<'\n';

      std_msgs::Int32 velocity_msg;
      velocity_msg.data = velocity_control;
      goal_velocity_pub_.publish(velocity_msg);

      is_motorState_updated_ = false;
    }
    ros::spinOnce();  // Handle callbacks
    rate.sleep();  // Maintain the desired loop rate
  }
}

void mySigintHandler(int sig)
{
  ROS_INFO("Killing Controller Node");
  ros::NodeHandle nh;

  // Setting Velocity to 0
  ros::Publisher goal_velocity_pub = nh.advertise<std_msgs::Int32>("/ankle_exo_frontal/motor/goal_velocity", 1);
  std_msgs::Int32 vel_msg;
  vel_msg.data = 0; // Set the Velocity control value to 0
  goal_velocity_pub.publish(vel_msg);

  // Disabling Motor Torque for Security
  ros::ServiceClient client = nh.serviceClient<ankle_exoskeleton::DynamixelCmdSimplified>("/ankle_exo_frontal/torque_enable");
  ankle_exoskeleton::DynamixelCmdSimplified srv;
  srv.request.id = 1;
  srv.request.value = 0;
  if (client.call(srv)) {
      ROS_INFO("Torque disabled in Motor ID : %d",srv.request.id);
  } else {
      ROS_ERROR("Failed to disable torque of Motor ID : %d",srv.request.id);
  }

  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_torque_based_vel_controller");
  DynamixelTorqueBasedVelocityController controller;

  signal(SIGINT, mySigintHandler); //Function to execute during Shutdown

  controller.run();

  return 0;
}
