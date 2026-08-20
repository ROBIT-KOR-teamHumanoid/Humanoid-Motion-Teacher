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

namespace humanoid {
DynamixelInit::DynamixelInit(std::string device_port, int baud_rate)
    : device_port_(device_port), baud_rate_(baud_rate) {}

DynamixelInit::~DynamixelInit() {
  if (portHandler_ != nullptr) {
    portHandler_->closePort();
  }
  portHandler_ = nullptr;
  packetHandler_ = nullptr;
}

bool DynamixelInit::init_sdk() {
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_port_.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (!portHandler_->openPort()) {
    throw std::runtime_error("Failed to open port");
  }

  if (!portHandler_->setBaudRate(baud_rate_)) {
    throw std::runtime_error("Failed to set baudrate");
  }
  return true;
}

bool DynamixelInit::ping_dynamixel(uint8_t id, uint16_t *model_number) {
  uint8_t dxl_error = 0;
  uint16_t detected_model = 0;
  int dxl_comm_result =
      packetHandler_->ping(portHandler_, id, &detected_model, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cout << "[Dynamixel] Ping failed for ID " << static_cast<int>(id)
              << std::endl;
    return false;
  } else {
    if (model_number != nullptr) *model_number = detected_model;
    std::cout << "[Dynamixel] Ping success for ID " << static_cast<int>(id)
              << ", model " << detected_model << std::endl;
  }
  return true;
}

}  // namespace humanoid
