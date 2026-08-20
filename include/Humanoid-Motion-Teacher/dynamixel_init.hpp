#ifndef HUMANOID_DYNAMIXEL_INIT_HPP_
#define HUMANOID_DYNAMIXEL_INIT_HPP_

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <iostream>
#include <string>
#include <vector>

constexpr int DEFAULT_BAUDRATE = 4500000;

namespace humanoid
{
class DynamixelInit
{
public:
  DynamixelInit(std::string device_port, int baud_rate = DEFAULT_BAUDRATE);
  ~DynamixelInit();

  bool init_sdk();
  bool ping_dynamixel(uint8_t id, uint16_t * model_number = nullptr);

private:
  std::string device_port_;
  int baud_rate_;

  dynamixel::PortHandler * portHandler_ = nullptr;
  dynamixel::PacketHandler * packetHandler_ = nullptr;
};

}  // namespace humanoid

#endif  // HUMANOID_DYNAMIXEL_INIT_HPP_
