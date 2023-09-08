/* Original Authors: Daniel Gomez-Vargas */

#include "../../include/ankle_exoskeleton/torque_based_vel_controller.h"


DxlTorqueBasedVelController::DxlTorqueBasedVelController()
  :node_handle_(""),
   priv_node_handle_("~")
{

  is_desired_value_updated_ = false;
  is_motorState_updated_ = false;

  desired_value_ = 0.0; // Initial desired value 0.0 Nm

  previous_time_ = ros::Time::now();
  previous_filter_value_ = 0.0;
}

DxlTorqueBasedVelController::~DxlTorqueBasedVelController(){}

bool DxlTorqueBasedVelController::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel;

  try {
      dynamixel = YAML::LoadFile(yaml_file.c_str());
      }
  catch (const YAML::Exception& e) {
      // Default Parameters for motor -> name:motor id:1
      ROS_WARN("Running without YAML file, setting ID: 1");
      dynamixel["motor"]["ID"] = 1;
      dynamixel["motor"]["Return_Delay_Time"] = 4;
    }

  if (dynamixel == NULL)
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }
    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
      {
        dynamixel_[name] = value;
        motor_id_ = value;
      }
        

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }

  return true;
}

void DxlTorqueBasedVelController::initSubscriber()
{
  for (auto const& dxl:dynamixel_)
  {
    std::string namespace_str = "";

    if (dynamixel_.size() != 1)
      namespace_str = std::string(dxl.first.c_str()); //TODO: Test for multiple dynamixel in same yaml file
    else
      namespace_str = priv_node_handle_.param<std::string>("name_space","");
    
    goal_desired_value_sub_ = priv_node_handle_.subscribe("/" + namespace_str + "/desired_torque", 1, &DxlTorqueBasedVelController::desiredValueCallback, this);
    dynamixel_status_sub_ = priv_node_handle_.subscribe("/" + namespace_str + "/status", 1, &DxlTorqueBasedVelController::dynamixelStatusCallback, this);
  
  }
}

void DxlTorqueBasedVelController::initPublisher()
{
  for (auto const& dxl:dynamixel_)
  {
    std::string namespace_str = "";

    if (dynamixel_.size() != 1)
      namespace_str = std::string(dxl.first.c_str()); //TODO: Test for multiple dynamixel in same yaml file
    else
      namespace_str = priv_node_handle_.param<std::string>("name_space","");
    
    goal_velocity_pub_ = priv_node_handle_.advertise<std_msgs::Int32>("/" + namespace_str + "/goal_velocity", 1);
    namespace_str_ = namespace_str; // Assigned to be used in other functions
  }
  
  }

void DxlTorqueBasedVelController::setMotorTorque(int value)
{
  for (auto const& dxl:dynamixel_)
  {
    std::string namespace_str = "";

    if (dynamixel_.size() != 1)
      namespace_str = std::string(dxl.first.c_str()); //TODO: Test for multiple dynamixel in same yaml file
    else
      namespace_str = priv_node_handle_.param<std::string>("name_space","");
    
    ros::ServiceClient client = priv_node_handle_.serviceClient<ankle_exoskeleton::DynamixelCmdSimplified>("/" + namespace_str + "/torque_enable");
    ankle_exoskeleton::DynamixelCmdSimplified srv;
    srv.request.id = (uint8_t)dxl.second;
    srv.request.value = value;
    if (client.call(srv)) {
        ROS_INFO("Torque = %d in Motor ID : %d", value, srv.request.id);
    } else {
        ROS_ERROR("Failed to call torque service in Motor ID : %d",srv.request.id);
    }
    
  }

  
}

void DxlTorqueBasedVelController::desiredValueCallback(const std_msgs::Float32::ConstPtr& msg)
{
  desired_value_ = msg->data;
  // is_desired_value_updated_ = true;
}

void DxlTorqueBasedVelController::dynamixelStatusCallback(const ankle_exoskeleton::DynamixelStatusList::ConstPtr& msg)
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

double DxlTorqueBasedVelController::lowPassFilter(double input)
{
  double cutoffFrequency = 100; // Hz

  ros::Time current_time = ros::Time::now();
  double deltaTime = (current_time - previous_time_).toSec();

  double alpha = 1.0 / (1.0 + 2.0 * M_PI * cutoffFrequency * deltaTime);

  double output = alpha * input + (1.0 - alpha) * previous_filter_value_;

  previous_filter_value_ = output;
  previous_time_ = current_time;

  return output;
}

void DxlTorqueBasedVelController::run()
{
  ros::Rate rate(600);
  ROS_INFO("Starting Controller ");
  // ros::Publisher filtered_pub = nh_.advertise<std_msgs::Float32>("/filtered_data", 1);
  // ros::Publisher unfiltered_pub = nh_.advertise<std_msgs::Float32>("/unfiltered_data", 1);

  while (!g_request_shutdown)
  {
    if (is_motorState_updated_)
    {
      // Torque Control based on Current Feedback (Low Level: Velocity Control)
      double Bv_gain = 0.02;

      double K_motor = 1.8;  // Nm/A
      double desired_current = (desired_value_ / K_motor); // Torque to Current

      double error_current = desired_current - present_current_;

      double u_velocity = error_current * 1/Bv_gain ;

      if (abs(u_velocity) < 3) // Tolerance of 3 rpm
        u_velocity = 0;

      if (u_velocity >= 55) // Motor Saturation in 55 rpm
        u_velocity = 55;
      else if (u_velocity <= -55)
        u_velocity = -55;

      // double u_velocity_filtered = lowPassFilter(u_velocity);

      // std_msgs::Float32 filtered_msg, unfiltered_msg;
      // filtered_msg.data = u_velocity_filtered;
      // unfiltered_msg.data = u_velocity;
      // filtered_pub.publish(filtered_msg);
      // unfiltered_pub.publish(unfiltered_msg);

      int32_t velocity_control = u_velocity/0.229; //u_velocity_filtered/0.229;
      // std::cout << "Error: " << error_current << "\tDesired Motor Velocity: " << u_velocity << "rpm" << "\tRegister Velocity: " << velocity_control <<'\n';

      velocity_msg_.data = velocity_control;
      goal_velocity_pub_.publish(velocity_msg_);

      is_motorState_updated_ = false;
    }
    ros::spinOnce();  // Handle callbacks
    rate.sleep();  // Maintain the desired loop rate
  }
  ROS_INFO("Killing Controller Node");
  velocity_msg_.data = 0;
  goal_velocity_pub_.publish(velocity_msg_);
  setMotorTorque(0);
  ros::shutdown();

}

// Replacement SIGINT handler
void mySigintHandler(int sig)
{
  g_request_shutdown = 1;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "dxl_torque_based_vel_controller");
  ros::NodeHandle node_handle("");

  DxlTorqueBasedVelController torque_controller;

  bool result = false;

  std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");


  result = torque_controller.getDynamixelsInfo(yaml_file);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return 0;
  }

  torque_controller.initPublisher();
  torque_controller.initSubscriber();
  torque_controller.setMotorTorque(1);

  signal(SIGINT, mySigintHandler); //Function to execute during Shutdown

  torque_controller.run();

  return 0;
}
