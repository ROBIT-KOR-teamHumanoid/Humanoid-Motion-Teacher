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

#include "../include/Humanoid-Motion-Teacher/dynamixel_ctrl.hpp"

namespace humanoid {

DynamixelCtrl::DynamixelCtrl(std::string device_port, int baud_rate,
                             std::vector<uint8_t> dynamixel_ids,
                             std::vector<uint16_t> model_numbers,
                             std::vector<std::string> model_names,
                             std::vector<DynamixelType> dynamixel_types,
                             std::vector<double> max_position_limit,
                             std::vector<double> min_position_limit)
    : device_port(device_port),
      baud_rate(baud_rate),
      dynamixel_ids(dynamixel_ids),
      model_numbers(model_numbers), model_names(model_names),
      dynamixel_types(dynamixel_types),
      max_position_limit(max_position_limit),
      min_position_limit(min_position_limit) {
  if (!init_dynamixel_sdk()) {
    std::cerr << "Failed to initialize Dynamixel SDK. Exiting..." << std::endl;
    throw std::runtime_error(
        "[DynamixelCtrl] Dynamixel SDK initialization failed");
  }
  // if (!init_max_velocity_limit()) {
  //   std::cerr << "Failed to initialize max velocity limit. Exiting..."
  //             << std::endl;
  //   throw std::runtime_error(
  //       "[DynamixelCtrl] Max velocity limit initialization failed");
  // }
  if (!init_indirect_address()) {
    std::cerr << "Failed to initialize indirect address. Exiting..."
              << std::endl;
    throw std::runtime_error(
        "[DynamixelCtrl] Indirect address initialization failed");
  }
  init_load_limits();
  controller_status = true;
}

DynamixelCtrl::~DynamixelCtrl() {
  if (portHandler_ != nullptr) {
    portHandler_->closePort();
    delete portHandler_;
    portHandler_ = nullptr;
  }
  // PacketHandler is an SDK singleton; deleting it breaks reconnection.
  packetHandler_ = nullptr;
}

void DynamixelCtrl::init_load_limits() {
  for (auto& motor : dynamixels) {
    if (!motor->has_current_feedback()) {
      motor->load_limit = 100.0;
      continue;
    }
    uint16_t raw_limit = 0;
    uint8_t error = 0;
    const int result = packetHandler_->read2ByteTxRx(
        portHandler_, motor->get_id(), EEPROM::CURRENT_LIMIT.first,
        &raw_limit, &error);
    if (result == COMM_SUCCESS && error == 0 && raw_limit > 0) {
      motor->load_limit = raw_limit * motor->dxl_current_ratio;
    }
  }
}

bool DynamixelCtrl::init_dynamixel_sdk() {
  std::cout << "========================================================"
            << std::endl;
  std::cout << "Initializing Dynamixel SDK" << std::endl;
  std::cout << "======================Configuration====================="
            << std::endl;
  std::cout << "Device Port: " << device_port << std::endl;
  std::cout << "Baud Rate: " << baud_rate << std::endl;
  if (dynamixel_ids.size() != dynamixel_types.size() ||
      dynamixel_ids.size() != model_numbers.size() ||
      dynamixel_ids.size() != model_names.size()) {
    std::cerr << "Dynamixel discovery data size mismatch" << std::endl;
    return false;
  }
  std::cout << "Dynamixel ID Count: " << dynamixel_ids.size() << std::endl;
  for (size_t i = 0; i < dynamixel_ids.size(); i++) {
    dynamixels.push_back(std::make_shared<Dynamixel>(
        dynamixel_types[i], dynamixel_ids[i], model_numbers[i], model_names[i],
        min_position_limit[i],
        max_position_limit[i]));
  }
  std::cout << "======================Port Handler====================="
            << std::endl;

  portHandler_ = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  std::cout << "Opening port: " << device_port << std::endl;
  if (!portHandler_->openPort()) {
    std::cerr << "Failed to open port" << std::endl;
    return false;
  }
  std::cout << "Port opened. Device Name: " << device_port << std::endl;

  std::cout << "Setting baudrate" << std::endl;
  if (!portHandler_->setBaudRate(baud_rate)) {
    std::cerr << "Failed to set baudrate" << std::endl;
    return false;
  }
  std::cout << "Baudrate set to: " << portHandler_->getBaudRate() << std::endl;

  return true;
}

bool DynamixelCtrl::init_max_velocity_limit() {
  int TOTAL_MOTOR = dynamixels.size();

  std::cout << "TOTAL_MOTOR : " << TOTAL_MOTOR << std::endl;

  for (int i = 0; i < TOTAL_MOTOR; i += 10) {
    dynamixel::GroupSyncRead SyncRead(portHandler_, packetHandler_,
                                      EEPROM::VELOCITY_LIMIT.first,
                                      EEPROM::VELOCITY_LIMIT.second);

    for (int j = 0; j < 10 && i + j < TOTAL_MOTOR; ++j) {
      bool dxl_addparam_result = SyncRead.addParam(dynamixels[i + j]->get_id());
      if (!dxl_addparam_result) {
        std::cerr << "2 Failed to add parameter to sync read on Dynamixel ID: "

                  << dynamixels[i + j]->get_id() << std::endl;
        return false;
      } else {
        std::cout
            << "Sucessfully added parameter to sync read on Dynamixel ID: "
            << dynamixels[i + j]->get_id() << std::endl;
      }
    }

    int dxl_comm_result = SyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      std::cerr << "Failed to get max velocity limit. Error Code: "
                << dxl_comm_result << " iteration : " << i << std::endl;

      return false;
    } else {
      std::cout << "Sucessfully got max velocity limit. iteration : " << i
                << std::endl;
    }

    SyncRead.clearParam();
  }

  // std::cout << "======================Initializing====================="
  //           << std::endl;

  // for (auto& dynamixel : dynamixels) {
  //   bool dxl_addparam_result = SyncRead.addParam(dynamixel->get_id());
  //   if (!dxl_addparam_result) {
  //     std::cerr << "2 Failed to add parameter to sync read on Dynamixel ID: "
  //               << dynamixel->get_id() << std::endl;
  //     return false;
  //   }
  // }

  // int dxl_comm_result = SyncRead.txRxPacket();
  // if (dxl_comm_result != COMM_SUCCESS) {
  //   std::cerr << "Failed to get max velocity limit. Error Code: "
  //             << dxl_comm_result << std::endl;
  //   return false;
  // } else {
  //   for (auto& dynamixel : dynamixels) {
  //     bool dxl_max_velocity_limit_result =
  //         dynamixel->get_max_velocity_limit(SyncRead);
  //     if (!dxl_max_velocity_limit_result) {
  //       std::cerr << "Failed to get max velocity limit on Dynamixel ID: "
  //                 << dynamixel->get_id() << std::endl;
  //       return false;
  //     } else {
  //       // std::cout << "Successfully got max velocity limit on Dynamixel ID:
  //       "
  //       // << dynamixel->get_id()
  //       //           << " Max Velocity Limit: " <<
  //       dynamixel->max_velocity_limit
  //       //           << std::endl;
  //     }
  //   }
  // }
  return true;
}

bool DynamixelCtrl::init_single_max_velocity_limit(uint8_t id) {
  dynamixel::GroupSyncRead SyncRead(portHandler_, packetHandler_,
                                    EEPROM::VELOCITY_LIMIT.first,
                                    EEPROM::VELOCITY_LIMIT.second);

  std::cout << "======================Initializing====================="
            << std::endl;

  bool dxl_addparam_result = SyncRead.addParam(id);
  if (!dxl_addparam_result) {
    std::cerr << "3 Failed to add parameter to sync read on Dynamixel ID: "
              << id << std::endl;
    return false;
  }

  int dxl_comm_result = SyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to get max velocity limit. Error Code: "
              << dxl_comm_result << std::endl;
    return false;
  } else {
    bool dxl_max_velocity_limit_result =
        dynamixels[id]->get_max_velocity_limit(SyncRead);
    if (!dxl_max_velocity_limit_result) {
      std::cerr << "Failed to get max velocity limit on Dynamixel ID: " << id
                << std::endl;
      return false;
    } else {
      std::cout << "Successfully got max velocity limit on Dynamixel ID: " << id
                << " Max Velocity Limit: " << dynamixels[id]->max_velocity_limit
                << std::endl;
    }
  }
  return true;
}

bool DynamixelCtrl::init_indirect_address() {
  // dynamixel::GroupBulkWrite BulkWrite(portHandler_, packetHandler_);
  // for (auto & dynamixel : dynamixels) {
  //   bool dxl_set_indirect_address_result =
  //   dynamixel->set_indirect_address(BulkWrite); if
  //   (!dxl_set_indirect_address_result) {
  //     std::cerr << "Failed to set indirect address on Dynamixel ID: " <<
  //     dynamixel->get_id()
  //               << std::endl;
  //     return false;
  //   } else {
  //     // std::cout << "Successfully set indirect address on Dynamixel ID: "
  //     << dynamixel->get_id()
  //     //           << std::endl;
  //   }
  // }

  // int dxl_comm_result = BulkWrite.txPacket();
  // if (dxl_comm_result != COMM_SUCCESS) {
  //   std::cerr << "2 Failed to bulk write" << std::endl;
  //   return false;
  // }

  int TOTAL_MOTOR = dynamixels.size();

  for (int i = 0; i < TOTAL_MOTOR; i += 5) {
    dynamixel::GroupBulkWrite bulkWrite(portHandler_, packetHandler_);

    for (int j = 0; j < 5 && i + j < TOTAL_MOTOR; ++j) {
      bool dxl_set_indirect_address_result =
          dynamixels[i + j]->set_indirect_address(bulkWrite);
      if (!dxl_set_indirect_address_result) {
        std::cerr << "Failed to set indirect address on Dynamixel ID: "
                  << dynamixels[i + j]->get_id() << std::endl;
        return false;
      }
    }

    int dxl_comm_result = bulkWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      std::cerr << "Failed to bulk write for batch starting with Dynamixel ID: "
                << dynamixels[i]->get_id() << std::endl;
      return false;
    }
  }
  return true;
}

bool DynamixelCtrl::init_single_indirect_address(uint8_t id) {
  dynamixel::GroupBulkWrite BulkWrite(portHandler_, packetHandler_);
  bool dxl_set_indirect_address_result =
      dynamixels[id]->set_indirect_address(BulkWrite);
  if (!dxl_set_indirect_address_result) {
    std::cerr << "Failed to set indirect address on Dynamixel ID: " << id
              << std::endl;
    return false;
  } else {
    std::cout << "Successfully set indirect address on Dynamixel ID: " << id
              << std::endl;
  }

  int dxl_comm_result = BulkWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to bulk write" << std::endl;
    return false;
  }
  return true;
}

bool DynamixelCtrl::set_torque(bool torque) {
  for (const uint16_t base : {static_cast<uint16_t>(224), static_cast<uint16_t>(634)}) {
    dynamixel::GroupSyncWrite sync_write(portHandler_, packetHandler_, base, 1);
    bool has_params = false;
    for (auto& motor : dynamixels) {
      if (motor->address(AddressNumber::TORQUE_ENABLE) != base) continue;
      has_params = true;
      if (!motor->set_torque(sync_write, torque)) {
        throw std::runtime_error("Failed to add torque for ID: " +
                                 std::to_string(motor->get_id()));
      }
    }
    if (has_params && sync_write.txPacket() != COMM_SUCCESS) {
      throw std::runtime_error("Failed to set torque");
    }
  }
  return true;
}

bool DynamixelCtrl::set_single_torque(uint8_t id, bool torque) {
  dynamixel::GroupSyncWrite SyncWrite(
      portHandler_, packetHandler_,
      dynamixels[id]->address(AddressNumber::TORQUE_ENABLE), 1);

  bool dxl_addparam_result = dynamixels[id]->set_torque(SyncWrite, torque);
  if (!dxl_addparam_result) {
    throw std::runtime_error(
        "Failed to add parameter to sync write on Dynamixel ID: " +
        std::to_string(id));
  }

  int dxl_comm_result = SyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
  } else {
    throw std::runtime_error("Failed to set torque");
  }
  return true;
}

bool DynamixelCtrl::read_dynamixel_status() {
  for (const uint16_t base : {static_cast<uint16_t>(224), static_cast<uint16_t>(634)}) {
    dynamixel::GroupSyncRead sync_read(portHandler_, packetHandler_, base, READ_LENGTH);
    bool has_params = false;
    for (auto& motor : dynamixels) {
      if (motor->address(AddressNumber::TORQUE_ENABLE) != base) continue;
      has_params = true;
      if (!sync_read.addParam(motor->get_id())) {
        throw std::runtime_error("Failed to add status read for ID: " +
                                 std::to_string(motor->get_id()));
      }
    }
    if (!has_params) continue;
    const int result = sync_read.txRxPacket();
    if (result != COMM_SUCCESS) {
      throw std::runtime_error("Failed to read Dynamixel status. Error Code: " +
                               std::to_string(result));
    }
    for (auto& motor : dynamixels) {
      if (motor->address(AddressNumber::TORQUE_ENABLE) == base &&
          !motor->get_dynamixel_status(sync_read)) {
        throw std::runtime_error("Failed to get status for ID: " +
                                 std::to_string(motor->get_id()));
        }
    }
  }
  return true;
}

void DynamixelCtrl::sync_write(std::vector<double> position,
                               std::vector<double> velocity,
                               std::vector<double> acceleration) {
  for (size_t i = 0; i < dynamixels.size(); i++) {
    dynamixels[i]->set_control_data(position[i], velocity[i], acceleration[i]);
  }
  for (const uint16_t base : {static_cast<uint16_t>(226), static_cast<uint16_t>(636)}) {
    dynamixel::GroupSyncWrite sync_write(portHandler_, packetHandler_, base, 12);
    bool has_params = false;
    for (auto& motor : dynamixels) {
      if (motor->address(AddressNumber::PROFILE_ACCELERATION) != base) continue;
      has_params = true;
      if (!motor->set_control_data_param(sync_write)) {
        throw std::runtime_error("Failed to add command for ID: " +
                                 std::to_string(motor->get_id()));
      }
    }
    if (has_params && sync_write.txPacket() != COMM_SUCCESS) {
      throw std::runtime_error("Failed to set goal position");
    }
  }
}

bool DynamixelCtrl::communication_cycle(
    const std::vector<double> *position, const std::vector<double> *velocity,
    const std::vector<double> *acceleration, bool force_read) {
  std::lock_guard<std::mutex> lock(communication_mutex_);
  if (position != nullptr) {
    if (velocity == nullptr || acceleration == nullptr) return false;
    sync_write(*position, *velocity, *acceleration);
  }
  if (force_read) {
    read_dynamixel_status();
    last_status_read_ = std::chrono::steady_clock::now();
  }
  return true;
}

std::vector<uint8_t> DynamixelCtrl::find_disconnected_ids() {
  std::lock_guard<std::mutex> lock(communication_mutex_);
  std::vector<uint8_t> missing;
  // Diagnose one ID at a time. With many actuators at 3 Mbps, Broadcast Ping
  // responses can overlap and produce COMM_RX_CORRUPT (-3002), making every
  // configured motor look disconnected.
  portHandler_->clearPort();
  for (const auto& motor : dynamixels) {
    bool responded = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      portHandler_->clearPort();
      uint8_t device_error = 0;
      uint16_t model_number = 0;
      const int result = packetHandler_->ping(
          portHandler_, motor->get_id(), &model_number, &device_error);
      // device_error reports an actuator hardware alert. It must not be used
      // as evidence of a broken serial connection: a valid status packet was
      // received whenever the communication result is COMM_SUCCESS.
      if (result == COMM_SUCCESS) {
        responded = true;
        break;
      }
    }
    if (!responded) {
      missing.push_back(static_cast<uint8_t>(motor->get_id()));
    }
  }
  portHandler_->clearPort();
  return missing;
}

void DynamixelCtrl::auto_reboot() {
  for (auto& dynamixel : dynamixels) {
    try {
      if (dynamixel->get_reboot_sequence() != DynamixelRebootSequence::STABLE) {
        // dynamixel->reboot_sequence(packetHandler_, portHandler_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } catch (const std::exception& e) {
      last_error_ = e.what();
      continue;
    }
  }
}

std::string DynamixelCtrl::get_reboot_sequence_str() const {
  std::string sequence;
  for (const auto& dynamixel : dynamixels) {
    sequence += "ID " + std::to_string(dynamixel->get_id()) + ": " +
                dynamixel->get_reboot_sequence_str() + ", ";
  }
  return sequence;
}

bool DynamixelCtrl::status() { return controller_status; }

}  // namespace humanoid
