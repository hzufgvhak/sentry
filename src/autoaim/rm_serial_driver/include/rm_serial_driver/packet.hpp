// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
#include <stdexcept>  

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  uint8_t task_mode : 2;     // 0-手打 1-自瞄 2-大符
  bool reset_tracker : 1;   // 1-重启跟踪器
  uint8_t is_play : 1;      // 1-play 0-stop
  bool change_target : 1;   // 1-change target
  uint8_t reserved : 2;

  uint8_t pose_state;   // 0-attack 1-defense 2-move

  float roll;
  float pitch;
  float yaw;
  uint16_t robot_HP;    // 机器人自身血量
  uint16_t game_time;  // (s) game time [0, 450]
  uint16_t checksum = 0;
} __attribute__((packed));


struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t state : 2;       // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t fire_flag : 1;  // 0-fired-off 1-fired-on
  uint8_t pose_state : 2;   // 0-attack 1-defense 2-move
  uint8_t reserved : 3;
  float pitch;
  float yaw;
  float nav_x;
  float nav_y;
  float nav_z;
  uint16_t checksum = 0;
} __attribute__((packed));
      
inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{ 
  if (data.size() < sizeof(ReceivePacket)) {
    throw std::runtime_error("ReceivePacket data too small: " + 
                             std::to_string(data.size()));
  }
  ReceivePacket packet;
  //std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  std::copy(data.begin(), data.begin() + sizeof(ReceivePacket),
            reinterpret_cast<uint8_t*>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_




// #ifndef RM_SERIAL_DRIVER__PACKET_HPP_
// #define RM_SERIAL_DRIVER__PACKET_HPP_

// #include <algorithm>
// #include <cmath>
// #include <cstdint>
// #include <vector>
// #include <stdexcept>
// #include <limits>  

// namespace rm_serial_driver
// {

// // 接收包保持不变（下位机发的，没问题）
// struct ReceivePacket
// {
//   uint8_t header = 0x5A;
//   uint8_t detect_color : 1;
//   uint8_t task_mode : 2;
//   bool reset_tracker : 1;
//   uint8_t is_play : 1;
//   bool change_target : 1;
//   uint8_t reserved : 2;
//   uint8_t pose_state;
//   float roll;
//   float pitch;
//   float yaw;
//   uint16_t robot_HP;
//   uint16_t game_time;
//   uint16_t checksum = 0;
// } __attribute__((packed));

// #pragma pack(push, 1)
// // 发送包 - 改用 uint8_t 替代位域，确保布局可控
// struct SendPacket
// {
//   uint8_t header = 0xA5;
//   uint8_t control_byte;  // 用位操作打包所有标志位
  
//   float pitch;
//   float yaw;
//   float nav_x;
//   float nav_y;
//   float nav_z;
//   uint16_t checksum = 0;
  
//   // 辅助函数：打包 control_byte
//   void packControlByte(uint8_t state, uint8_t fire_flag, uint8_t pose_state, uint8_t reserved = 0) {
//     // state: 2bits, fire_flag: 1bit, pose_state: 2bits, reserved: 3bits
//     control_byte = (state & 0x03) | 
//                    ((fire_flag & 0x01) << 2) | 
//                    ((pose_state & 0x03) << 3) | 
//                    ((reserved & 0x07) << 5);
//   }
  
//   // 辅助函数：解包（调试用）
//   void unpackControlByte(uint8_t& state, uint8_t& fire_flag, uint8_t& pose_state, uint8_t& reserved) const {
//     state = control_byte & 0x03;
//     fire_flag = (control_byte >> 2) & 0x01;
//     pose_state = (control_byte >> 3) & 0x03;
//     reserved = (control_byte >> 5) & 0x07;
//   }

//   // 设置导航坐标（带有效性检查，防止inf/nan）
//   // 同时限制在合理范围内: x,y [-50,50]m, z [-10,10]m
//   void setNav(float x, float y, float z) {
//     nav_x = clampNav(x, -50.0f, 50.0f);
//     nav_y = clampNav(y, -50.0f, 50.0f);
//     nav_z = clampNav(z, -10.0f, 10.0f);
//   }

//   // 设置瞄准角度（带有效性检查）
//   // 限制角度范围: [-π, π]
//   void setAim(float p, float y) {
//     pitch = clampAngle(p);
//     yaw = clampAngle(y);
//   }

//   // 统一设置所有pose数据
//   void setPose(float p, float y, float nx, float ny, float nz) {
//     setAim(p, y);
//     setNav(nx, ny, nz);
//   }

//   // 检查数据包是否有效（发送前调用）
//   bool isValid() const {
//     return std::isfinite(pitch) && std::isfinite(yaw) &&
//            std::isfinite(nav_x) && std::isfinite(nav_y) && std::isfinite(nav_z);
//   }

// private:
//   // 导航坐标有效性检查：过滤inf/nan并限制范围
//   float clampNav(float val, float min_val, float max_val) const {
//     if (!std::isfinite(val)) {
//       return 0.0f;
//     }
//     return std::clamp(val, min_val, max_val);
//   }

//   // 角度有效性检查：限制在[-π, π]
//   float clampAngle(float val) const {
//     if (!std::isfinite(val)) {
//       return 0.0f;
//     }
//     // 限制角度范围 [-π, π]
//     const float PI = 3.14159265358979f;
//     while (val > PI) val -= 2.0f * PI;
//     while (val < -PI) val += 2.0f * PI;
//     return val;
//   }
//  }; 
//  #pragma pack(pop)
// // 验证大小
// static_assert(sizeof(SendPacket) == 1 + 1 + 4*5 + 2, "SendPacket size mismatch! Expected 24 bytes");
// static_assert(sizeof(ReceivePacket) == 1 + 1 + 1 + 4*3 + 2*3, "ReceivePacket size mismatch!");


// inline ReceivePacket fromVector(const std::vector<uint8_t>& data)
// { 
//   if (data.size() < sizeof(ReceivePacket)) {
//     throw std::runtime_error("ReceivePacket data too small: " + std::to_string(data.size()) + 
//                              ", expected: " + std::to_string(sizeof(ReceivePacket)));
//   }
//   ReceivePacket packet;
//   std::copy(data.begin(), data.begin() + sizeof(ReceivePacket),
//             reinterpret_cast<uint8_t*>(&packet));
//   return packet;
// }

// inline std::vector<uint8_t> toVector(const SendPacket& data)
// {
//   std::vector<uint8_t> packet(sizeof(SendPacket));
//   std::copy(reinterpret_cast<const uint8_t*>(&data),
//             reinterpret_cast<const uint8_t*>(&data) + sizeof(SendPacket), 
//             packet.begin());
//   return packet;
// }

// }  // namespace rm_serial_driver

// #endif  // RM_SERIAL_DRIVER__PACKET_HPP_


