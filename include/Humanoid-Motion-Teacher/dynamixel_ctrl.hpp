/*
 * Copyright 2024 Myeong Jin Lee
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
 */

#ifndef HUMANOID_DYNAMIXEL_CTRL_HPP_
#define HUMANOID_DYNAMIXEL_CTRL_HPP_

#include "Humanoid-Motion-Teacher/dynamixel.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// #include "dynamixel_rdk_msgs/msg/dynamixel_bulk_read_msgs.hpp"
// #include "dynamixel_rdk_msgs/msg/dynamixel_control_msgs.hpp"

#include <iostream>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace humanoid
{
class DynamixelCtrl
{
public:  // Public Functions
  DynamixelCtrl(
    std::string device_port, int baud_rate, std::vector<uint8_t> dynamixel_ids,
    std::vector<uint16_t> model_numbers, std::vector<std::string> model_names,
    std::vector<DynamixelType> dynamixel_types, std::vector<double> max_position_limit,
    std::vector<double> min_position_limit);
  ~DynamixelCtrl();

  bool init_single_max_velocity_limit(uint8_t id);
  bool init_single_indirect_address(uint8_t id);

  bool set_torque(bool torque);
  bool set_single_torque(uint8_t id, bool torque);

  bool read_dynamixel_status();
  bool communication_cycle(
    const std::vector<double> * position = nullptr,
    const std::vector<double> * velocity = nullptr,
    const std::vector<double> * acceleration = nullptr, bool force_read = false);
  void sync_write(
    std::vector<double> position, std::vector<double> velocity, std::vector<double> acceleration);

  void auto_reboot();

  std::string get_reboot_sequence_str() const;
  std::vector<uint8_t> find_disconnected_ids();
  std::string get_last_error() const { return last_error_; }

public:  // Public Variables
  bool controller_status;

  std::vector<std::shared_ptr<Dynamixel>> dynamixels;

private:  // Private Functions
  bool init_dynamixel_sdk();
  bool init_max_velocity_limit();
  bool init_indirect_address();
  void init_load_limits();

  bool status();

private:  // Private Variables
  std::string device_port;
  int baud_rate;
  std::vector<uint8_t> dynamixel_ids;
  std::vector<uint16_t> model_numbers;
  std::vector<std::string> model_names;
  std::vector<DynamixelType> dynamixel_types;
  std::vector<double> max_position_limit;
  std::vector<double> min_position_limit;
  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;

  std::string last_error_;
  std::chrono::steady_clock::time_point last_status_read_{};
  std::mutex communication_mutex_;
};
}  // namespace humanoid

#endif  // DYNAMIXEL_RDK_ROS_DYNAMIXEL_CTRL_HPP_
