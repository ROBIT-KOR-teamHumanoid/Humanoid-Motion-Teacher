#ifndef HUMANOID_DYNAMIXEL_INIT_HPP_
#define HUMANOID_DYNAMIXEL_INIT_HPP_

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <iostream>
#include <string>
#include <vector>

#define DEFAULT_BAUDRATE 4500000
#define SEARCH_ID_END 254

namespace humanoid
{
class DynamixelInit
{
public:
  DynamixelInit(std::string device_port);
  ~DynamixelInit();

  bool init_sdk();
  bool ping_dynamixel(uint8_t id);

private:
  std::string device_port_;

  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;
};

}  // namespace humanoid

#endif  // HUMANOID_DYNAMIXEL_INIT_HPP_
