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

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_CONTROLLERS_H
#define DYNAMIXEL_WORKBENCH_CONTROLLERS_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <yaml-cpp/yaml.h>

#include "../dynamixel_workbench/dynamixel_workbench.h"
#include <ankle_exoskeleton/DynamixelStatusList.h>
#include <ankle_exoskeleton/DynamixelCommand.h> 
#include <ankle_exoskeleton/DynamixelCmdSimplified.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT_PWM_VOLTAGE_TEMPERATURE 0

// #define DEBUG

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_status_list_pub_;
  
  // ROS Topic Subscriber
  ros::Subscriber cmd_current_sub_;
  ros::Subscriber cmd_velocity_sub_;
  ros::Subscriber cmd_position_sub_;
  ros::Subscriber cmd_pwm_sub_;  
  
  // ROS Service Server
  ros::ServiceServer dynamixel_command_server_;
  ros::ServiceServer torque_enable_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  ankle_exoskeleton::DynamixelStatusList dynamixel_status_list_;

  bool is_server_activated_;

  std::string serial_name; 


  double read_period_;
  double pub_period_;


 public:
  DynamixelController();
  ~DynamixelController();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);

  double getReadPeriod(){return read_period_;}
  double getPublishPeriod(){return pub_period_;}

  void initPublisher(void);
  void initSubscriber(const std::string control_mode);

  void initServer();
  
  void commandCurrentCallback(const std_msgs::Int32::ConstPtr &msg);
  void commandVelocityCallback(const std_msgs::Int32::ConstPtr &msg);
  void commandPositionCallback(const std_msgs::Int32::ConstPtr &msg);
  void commandPWMCallback(const std_msgs::Int32::ConstPtr &msg);
  void readCallback(const ros::TimerEvent&);
  void publishCallback(const ros::TimerEvent&);

  bool dynamixelCommandMsgCallback(ankle_exoskeleton::DynamixelCommand::Request &req,
                                   ankle_exoskeleton::DynamixelCommand::Response &res);
  bool torqueEnableMsgCallback(ankle_exoskeleton::DynamixelCmdSimplified::Request &req,
                               ankle_exoskeleton::DynamixelCmdSimplified::Response &res);  
                               
  void disableTorque();     
  void enableTorque();    
  void setControlMode(const std::string control_mode);       

};

#endif //DYNAMIXEL_WORKBENCH_CONTROLLERS_H
