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

#include "../include/Humanoid-Motion-Teacher/dynamixel_init.hpp"

namespace humanoid
{
DynamixelInit::DynamixelInit(std::string device_port) : device_port_(device_port)
{
}

DynamixelInit::~DynamixelInit()
{
  portHandler_->closePort();
  portHandler_ = nullptr;
  packetHandler_ = nullptr;
}

bool DynamixelInit::init_sdk()
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_port_.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (!portHandler_->openPort()) {
    throw std::runtime_error("Failed to open port");
  }

  if (!portHandler_->setBaudRate(DEFAULT_BAUDRATE)) {
    throw std::runtime_error("Failed to set baudrate");
  }
  return true;
}

bool DynamixelInit::ping_dynamixel(uint8_t id)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packetHandler_->ping(portHandler_, id, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    return false;
  }
  return true;
}

}  // namespace humanoid
