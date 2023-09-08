/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Original Authors: Taehun Lim (Darby) */

/* Modified Version Author: Daniel Gomez-Vargas */

#include "../../include/ankle_exoskeleton/dynamixel_controllers.h"


DynamixelController::DynamixelController()
  :node_handle_(""),
   priv_node_handle_("~"),
   is_server_activated_(false)
{
  is_server_activated_ = priv_node_handle_.param<bool>("use_dynamixel_server", false);

  read_period_ = priv_node_handle_.param<double>("dxl_read_period", 0.0025f);
  pub_period_ = priv_node_handle_.param<double>("publish_period", 0.0025f);

  dxl_wb_ = new DynamixelWorkbench;
}

DynamixelController::~DynamixelController(){}

bool DynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
  serial_name = port_name; // Class variable to ROS_INFO visualization in other functions

  return result;
}

bool DynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel;

  try {
      dynamixel = YAML::LoadFile(yaml_file.c_str());
      }
  catch (const YAML::Exception& e) {
      // Default Parameters for motor -> name: motor id:1
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
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }

  return true;
}

bool DynamixelController::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {
      ROS_INFO("Name : %s, ID : %d, Model Number : %d, Protocol : %.1f", dxl.first.c_str(), dxl.second, model_number,dxl_wb_->getProtocolVersion());
    }
  }

  return result;
}

bool DynamixelController::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          result = 1;
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}

bool DynamixelController::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  // const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  // if (goal_position == NULL) return false;
  //
  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)  goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL)  return false;
  //
  // const ControlItem *goal_current = dxl_wb_->getItemInfo(it->second, "Goal_Current");
  // if (goal_current == NULL) return false;
  //
  // const ControlItem *goal_pwm = dxl_wb_->getItemInfo(it->second, "Goal_PWM");
  // if (goal_pwm == NULL) return false;

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    const ControlItem *present_pwm = dxl_wb_->getItemInfo(it->second, "Present_PWM");
    if (present_pwm == NULL) return false;
    control_items_["Present_PWM"] = present_pwm;
  }

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)
  {
    present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
    ROS_INFO("Current has been set with the Present Load register ");
  }
  //if (present_current == NULL)  present_current = dxl_wb_->getItemInfo(it->second, "Current");
  if (present_current == NULL) return false;

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)  present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_input_voltage = dxl_wb_->getItemInfo(it->second, "Present_Input_Voltage");
  if (present_input_voltage == NULL) present_input_voltage = dxl_wb_->getItemInfo(it->second, "Present_Voltage");
  if (present_input_voltage == NULL) return false;

  const ControlItem *present_temperature = dxl_wb_->getItemInfo(it->second, "Present_Temperature");
  if (present_temperature == NULL) return false;

  // control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;
  // control_items_["Goal_Current"] = goal_current;
  // control_items_["Goal_PWM"] = goal_pwm;

  control_items_["Present_Current"] = present_current;
  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Input_Voltage"] = present_input_voltage;
  control_items_["Present_Temperature"] = present_temperature;

  return true;
}

bool DynamixelController::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  // auto it = dynamixel_.begin();

  // result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  // if (result == false)
  // {
  //   ROS_ERROR("%s", log);
  //   return result;
  // }
  // else
  // {
  //   ROS_INFO("Goal Position: %s", log);
  // }
  //
  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("Goal Velocity: %s", log);
  }
  //
  // result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
  // if (result == false)
  // {
  //   ROS_ERROR("%s", log);
  //   return result;
  // }
  // else
  // {
  //   ROS_INFO("Goal Current: %s", log);
  // }
  //
  // result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_PWM"]->address, control_items_["Goal_PWM"]->data_length, &log);
  // if (result == false)
  // {
  //   ROS_ERROR("%s", log);
  //   return result;
  // }
  // else
  // {
  //   ROS_INFO("Goal PWM: %s", log);
  // }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_PWM"]->address);
    // Considering Temperature and Input Voltage values, the read_length include +11 to reach the address 146
    uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + control_items_["Present_PWM"]->data_length+11;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }else if(dxl_wb_->getProtocolVersion() == 1.0f){
    result = true;
  }

  return result;
}

void DynamixelController::initPublisher()
{
  dynamixel_status_list_pub_ = priv_node_handle_.advertise<ankle_exoskeleton::DynamixelStatusList>("status", 2);
}

void DynamixelController::initSubscriber(const std::string control_mode)
{
  for (auto const& dxl:dynamixel_)
  {
    std::string topic_name = "";

    if (dynamixel_.size() != 1)
      topic_name = std::string(dxl.first.c_str()); //TODO: Test for multiple dynamixel in same yaml file
    else
      topic_name = priv_node_handle_.getNamespace();

    if (control_mode == "current_control")
    {
      topic_name = topic_name + "/goal_current";
      cmd_current_sub_ = priv_node_handle_.subscribe(topic_name, 2, &DynamixelController::commandCurrentCallback, this);
    }
    else if (control_mode == "velocity_control")
    {
      topic_name = topic_name + "/goal_velocity";
      cmd_velocity_sub_ = priv_node_handle_.subscribe(topic_name, 2, &DynamixelController::commandVelocityCallback, this);
    }
    else if (control_mode == "position_control")
    {
      topic_name = topic_name + "/goal_position";
      cmd_position_sub_ = priv_node_handle_.subscribe(topic_name, 2, &DynamixelController::commandPositionCallback, this);
    }
    else if (control_mode == "pwm_control")
    {
      topic_name = topic_name + "/goal_pwm";
      cmd_pwm_sub_ = priv_node_handle_.subscribe(topic_name, 2, &DynamixelController::commandPWMCallback, this);
    }
  }
}

void DynamixelController::initServer()
{
  // TODO: Services for Controllers' Gains Registers
  if (is_server_activated_) dynamixel_command_server_ = priv_node_handle_.advertiseService("dynamixel_command", &DynamixelController::dynamixelCommandMsgCallback, this);
  torque_enable_server_ = priv_node_handle_.advertiseService("torque_enable", &DynamixelController::torqueEnableMsgCallback, this);
}

void DynamixelController::commandCurrentCallback(const std_msgs::Int32::ConstPtr &msg)
{
  bool result = false;
  const char* log = NULL;

  int32_t goal_curent = msg->data;
  result = dxl_wb_->itemWrite(1,"Goal_Current",goal_curent, &log);
}

void DynamixelController::commandVelocityCallback(const std_msgs::Int32::ConstPtr &msg)
{
  bool result = false;
  const char* log = NULL;

  for (auto const& dxl:dynamixel_)
  {
    int32_t goal_vel = msg->data;
    uint8_t id = (uint8_t)dxl.second;
    result = dxl_wb_->itemWrite(id,"Goal_Velocity",goal_vel, &log);
  }
}

void DynamixelController::commandPositionCallback(const std_msgs::Int32::ConstPtr &msg)
{
  bool result = false;
  const char* log = NULL;

  int32_t goal_pos = msg->data;
  result = dxl_wb_->itemWrite(1,"Goal_Position",goal_pos, &log);
}

void DynamixelController::commandPWMCallback(const std_msgs::Int32::ConstPtr &msg)
{
  bool result = false;
  const char* log = NULL;

  int32_t goal_pwm = msg->data;
  result = dxl_wb_->itemWrite(1,"Goal_PWM",msg->data, &log);
}

void DynamixelController::readCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_read_secs =ros::Time::now().toSec();
#endif
  bool result = false;
  const char* log = NULL;

  ankle_exoskeleton::DynamixelStatus  dynamixel_status[dynamixel_.size()];
  dynamixel_status_list_.dynamixel_status.clear();
  int32_t get_pwm[dynamixel_.size()];
  int32_t get_current[dynamixel_.size()];
  int32_t get_velocity[dynamixel_.size()];
  int32_t get_position[dynamixel_.size()];
  int32_t get_voltage[dynamixel_.size()];
  int32_t get_temperature[dynamixel_.size()];

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;
  for (auto const& dxl:dynamixel_)
  {
    dynamixel_status[id_cnt].name = dxl.first;
    dynamixel_status[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }
  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT_PWM_VOLTAGE_TEMPERATURE,
                                id_array,
                                dynamixel_.size(),
                                &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT_PWM_VOLTAGE_TEMPERATURE,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_PWM"]->address,
                                                  control_items_["Present_PWM"]->data_length,
                                                  get_pwm,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }


    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT_PWM_VOLTAGE_TEMPERATURE,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Current"]->address,
                                                  control_items_["Present_Current"]->data_length,
                                                  get_current,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT_PWM_VOLTAGE_TEMPERATURE,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Velocity"]->address,
                                                  control_items_["Present_Velocity"]->data_length,
                                                  get_velocity,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT_PWM_VOLTAGE_TEMPERATURE,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Position"]->address,
                                                  control_items_["Present_Position"]->data_length,
                                                  get_position,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT_PWM_VOLTAGE_TEMPERATURE,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Input_Voltage"]->address,
                                                  control_items_["Present_Input_Voltage"]->data_length,
                                                  get_voltage,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT_PWM_VOLTAGE_TEMPERATURE,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Temperature"]->address,
                                                  control_items_["Present_Temperature"]->data_length,
                                                  get_temperature,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    for(uint8_t index = 0; index < id_cnt; index++)
    {
      dynamixel_status[index].present_pwm = get_pwm[index];
      dynamixel_status[index].present_current = get_current[index];
      dynamixel_status[index].present_velocity = get_velocity[index];
      dynamixel_status[index].present_position = get_position[index];
      dynamixel_status[index].present_input_voltage = get_voltage[index];
      dynamixel_status[index].present_temperature = get_temperature[index];

      dynamixel_status_list_.dynamixel_status.push_back(dynamixel_status[index]);
    }
  }
  else if(dxl_wb_->getProtocolVersion() == 1.0f)
  {
    uint16_t length_of_data = control_items_["Present_Position"]->data_length +
                              control_items_["Present_Velocity"]->data_length +
                              2 + //Present Load Register
                              control_items_["Present_Input_Voltage"]->data_length +
                              control_items_["Present_Temperature"]->data_length +
                              // 7 + //Other registers to read Current Register
                              // 16 + //No registers to read Current Register
                              control_items_["Present_Current"]->data_length;
    uint32_t get_all_data[length_of_data];
    uint8_t dxl_cnt = 0;
    for (auto const& dxl:dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                     control_items_["Present_Position"]->address,
                                     length_of_data,
                                     get_all_data,
                                     &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }
      // dynamixel_status[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[29], get_all_data[30]); To read Current Register
      dynamixel_status[dxl_cnt].present_temperature = get_all_data[7];
      dynamixel_status[dxl_cnt].present_input_voltage = get_all_data[6];
      dynamixel_status[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[4], get_all_data[5]); //Present Load Register
      dynamixel_status[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
      dynamixel_status[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);


      dynamixel_status_list_.dynamixel_status.push_back(dynamixel_status[dxl_cnt]);
      dxl_cnt++;
    }
  }

#ifdef DEBUG
  ROS_WARN("[readCallback] diff_secs : %f", ros::Time::now().toSec() - priv_read_secs);
  priv_read_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::publishCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_pub_secs =ros::Time::now().toSec();
#endif
  try {
      dynamixel_status_list_pub_.publish(dynamixel_status_list_);
  }
  catch (const std::exception& e) {
    // Exception handling code
    ROS_ERROR("Exception caught: %s",e.what());
  }

#ifdef DEBUG
  ROS_WARN("[publishCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
  priv_pub_secs = ros::Time::now().toSec();
#endif
}


bool DynamixelController::dynamixelCommandMsgCallback(ankle_exoskeleton::DynamixelCommand::Request &req,
                                                      ankle_exoskeleton::DynamixelCommand::Response &res)
{
  bool result = false;
  const char* log;

  uint8_t id = req.id;
  std::string item_name = req.addr_name;
  int32_t value = req.value;

  result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
  }

  res.comm_result = result;

  return true;
}

bool DynamixelController::torqueEnableMsgCallback(ankle_exoskeleton::DynamixelCmdSimplified::Request &req,
                                                  ankle_exoskeleton::DynamixelCmdSimplified::Response &res)
{
  bool result = false;
  const char* log;

  uint8_t id = req.id;
  std::string item_name = "Torque_Enable";
  int32_t value = req.value;

  if (id == 0)
  {
    if (value == 1)
    {
      ROS_INFO("Enabling torque for connected motors of %s", serial_name.c_str());
      enableTorque();
    }
    else
    {
      ROS_INFO("Disabling torque for connected motors of %s", serial_name.c_str());
      disableTorque();
    }
    res.comm_result = true;
    return true;
  }
  else
  {
    result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
    }

    res.comm_result = result;

    return true;
  }
  
}

void DynamixelController::disableTorque()
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint8_t id = (uint8_t)dxl.second;
    std::string item_name = "Torque_Enable";
    int32_t value = 0;

    result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
    if (result == false)
    {
      ROS_ERROR("Failed to disable Torque in Motor ID : %d",id);
    }
  }
}

void DynamixelController::enableTorque()
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint8_t id = (uint8_t)dxl.second;
    std::string item_name = "Torque_Enable";
    int32_t value = 1;

    result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
    if (result == false)
    {
      ROS_ERROR("Failed to enable Torque in Motor ID : %d",id);
    }
  }
}

void DynamixelController::setControlMode(const std::string control_mode)
{
  bool result = false;
  const char* log;

  disableTorque();
  for (auto const& dxl:dynamixel_)
  {
    // TODO: Include Extended Controller, Current-based, Velocity is not working in protocol 1.0
    uint8_t id = (uint8_t)dxl.second;
    if (control_mode == "current_control")
    {
      result = dxl_wb_->setCurrentControlMode(id, &log);
      if (result == false) ROS_ERROR("%s", log);
      else ROS_INFO("Current Control set in Motor ID : %d", id);
    }
    else if (control_mode == "velocity_control")
    {
      result = dxl_wb_->setVelocityControlMode(id, &log);
      if (result == false) ROS_ERROR("%s", log);
      else ROS_INFO("Velocity Control set in Motor ID : %d", id);
    }
    else if (control_mode == "position_control")
    {
      result = dxl_wb_->setPositionControlMode(id, &log);
      if (result == false) ROS_ERROR("%s", log);
      else ROS_INFO("Position Control set in Motor ID : %d", id);
    }
    else if (control_mode == "pwm_control")
    {
      result = dxl_wb_->setPWMControlMode(id, &log);
      if (result == false) ROS_ERROR("%s", log);
      else ROS_INFO("PWM Control set in Motor ID : %d", id);
    }
    else if (control_mode.empty() == false)
    {
      ROS_ERROR("Desired Controller: \"%s\" is not available \nAvailable Controllers: current_control, velocity_control, position_control, pwm_control or empty for no controller", control_mode.c_str());
      result = dxl_wb_->itemWrite(id, "Torque_Enable", 0, &log);
      if (result == false) ROS_ERROR("%s", log);
      return;
    }
    else
    {
      ROS_INFO("No controller was selected");
      result = dxl_wb_->itemWrite(id, "Torque_Enable", 0, &log);
      if (result == false) ROS_ERROR("%s", log);
      return;
    }
    result = dxl_wb_->itemWrite(id, "Torque_Enable", 1, &log);
    if (result == false) ROS_ERROR("%s", log);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_controllers");
  ros::NodeHandle node_handle("");

  std::string port_name = "/dev/ttyUSB0";
  uint32_t baud_rate = 4000000;
  std::string control_mode = ""; //current_control, velocity_control, position_control or pwm_control

  if (argc < 4)
  {
    ROS_WARN("Using default parameters without controller: Port name: %s and Baud Rate: %d",port_name.c_str(),baud_rate);
    ROS_INFO("To change include '-port_name','-baud_rate','-controller' arguments for connected Dynamixels");
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
    try
    {
      control_mode = argv[3];
    } catch (const std::exception& ex) {
      control_mode = "";
      ROS_WARN("No controller selected");
    }

  }

  DynamixelController dynamixel_controller;

  bool result = false;

  std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

  result = dynamixel_controller.initWorkbench(port_name, baud_rate);
  if (result == false)
  {
    ROS_ERROR("Please check USB port name");
    return 0;
  }

  result = dynamixel_controller.getDynamixelsInfo(yaml_file);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return 0;
  }

  result = dynamixel_controller.loadDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check Dynamixel ID or BaudRate");
    return 0;
  }

  result = dynamixel_controller.initDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
    return 0;
  }

  result = dynamixel_controller.initControlItems();
  if (result == false)
  {
    ROS_ERROR("Please check control items");
    return 0;
  }

  result = dynamixel_controller.initSDKHandlers();
  if (result == false)
  {
    ROS_ERROR("Failed to set Dynamixel SDK Handler");
    return 0;
  }

  dynamixel_controller.initPublisher();
  dynamixel_controller.initSubscriber(control_mode);
  dynamixel_controller.initServer();

  dynamixel_controller.setControlMode(control_mode);

  ROS_INFO("Starting Dynamixel Controllers");

  ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()), &DynamixelController::readCallback, &dynamixel_controller);
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()), &DynamixelController::publishCallback, &dynamixel_controller);

  ros::spin();

  return 0;
}
