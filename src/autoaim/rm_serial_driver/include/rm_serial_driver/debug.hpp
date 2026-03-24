// // rm_serial_driver_debug.hpp
// #ifndef RM_SERIAL_DRIVER__DEBUG_HPP_
// #define RM_SERIAL_DRIVER__DEBUG_HPP_

// #include <sstream>
// #include <iomanip>
// #include <bitset>
// #include <cstdint>
// #include <vector>

// #include "rm_serial_driver/packet.hpp"

// namespace rm_serial_driver
// {

// class PacketDebugger
// {
// public:
//   // 打印完整数据包
//   static std::string debugSendPacket(const SendPacket& packet)
//   {
//     std::stringstream ss;
    
//     ss << "========== SendPacket Debug ==========\n";
    
//     // 1. 打印基本字段
//     ss << "[Header] 0x" << std::hex << std::setw(2) << std::setfill('0') 
//        << (int)packet.header << std::dec << "\n";
    
//     // 2. 解包并打印 control_byte
//     uint8_t state, fire_flag, pose_state, reserved;
//     packet.unpackControlByte(state, fire_flag, pose_state, reserved);
    
//     ss << "[Control Byte] 0x" << std::hex << std::setw(2) << std::setfill('0') 
//        << (int)packet.control_byte << std::dec << "\n";
//     ss << "  ├─ state      (bit0-1): " << (int)state 
//        << " [" << stateToString(state) << "]\n";
//     ss << "  ├─ fire_flag  (bit2)  : " << (int)fire_flag 
//        << " [" << (fire_flag ? "FIRE" : "STOP") << "]\n";
//     ss << "  ├─ pose_state (bit3-4): " << (int)pose_state 
//        << " [" << poseToString(pose_state) << "]\n";
//     ss << "  └─ reserved   (bit5-7): " << (int)reserved << "\n";
    
//     // 3. 打印二进制位
//     std::bitset<8> bits(packet.control_byte);
//     ss << "  Binary: " << bits.to_string() << "\n";
//     ss << "          ↑↑↑↑↑↑↑↑\n";
//     ss << "          76543210 (bit位)\n";
    
//     // 4. 打印float数据
//     ss << std::fixed << std::setprecision(3);
//     ss << "[Float Data]\n";
//     ss << "  ├─ pitch : " << std::setw(8) << packet.pitch << " rad (" 
//        << packet.pitch * 180.0 / M_PI << " deg)\n";
//     ss << "  ├─ yaw   : " << std::setw(8) << packet.yaw << " rad (" 
//        << packet.yaw * 180.0 / M_PI << " deg)\n";
//     ss << "  ├─ nav_x : " << std::setw(8) << packet.nav_x << "\n";
//     ss << "  ├─ nav_y : " << std::setw(8) << packet.nav_y << "\n";
//     ss << "  └─ nav_z : " << std::setw(8) << packet.nav_z << "\n";
    
//     // 5. 打印校验和
//     ss << "[Checksum] 0x" << std::hex << std::setw(4) << std::setfill('0') 
//        << packet.checksum << std::dec << "\n";
    
//     return ss.str();
//   }
  
//   // 打印原始字节流
//   static std::string debugRawBytes(const std::vector<uint8_t>& data)
//   {
//     std::stringstream ss;
//     ss << "========== Raw Bytes (" << data.size() << " bytes) ==========\n";
    
//     // 十六进制
//     ss << "Hex:    ";
//     for (size_t i = 0; i < data.size(); ++i) {
//       ss << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
//       if ((i + 1) % 8 == 0) ss << " ";
//       if ((i + 1) % 16 == 0 && i + 1 < data.size()) ss << "\n        ";
//     }
//     ss << "\n";
    
//     // 带索引的详细解析
//     ss << std::dec;
//     ss << "\nDetailed:\n";
//     ss << "  [00] header:       0x" << std::hex << (int)data[0] << "\n";
//     ss << "  [01] control:      0x" << (int)data[1] 
//        << " (state=" << (data[1] & 0x03) 
//        << ", fire=" << ((data[1] >> 2) & 0x01) 
//        << ", pose=" << ((data[1] >> 3) & 0x03) << ")\n";
    
//     // float解析（小端序）
//     auto read_float = [&data](size_t offset) -> float {
//       if (offset + 4 > data.size()) return 0.0f;
//       uint32_t u = data[offset] | (data[offset+1] << 8) | 
//                    (data[offset+2] << 16) | (data[offset+3] << 24);
//       float f;
//       std::memcpy(&f, &u, sizeof(f));
//       return f;
//     };
    
//     ss << std::fixed << std::setprecision(3);
//     ss << "  [02-05] pitch:    " << read_float(2) << " (bytes: ";
//     for (int i = 2; i <= 5; ++i) ss << std::hex << std::setw(2) << (int)data[i] << " ";
//     ss << ")\n";
    
//     ss << "  [06-09] yaw:      " << std::dec << read_float(6) << " (bytes: ";
//     for (int i = 6; i <= 9; ++i) ss << std::hex << std::setw(2) << (int)data[i] << " ";
//     ss << ")\n";
    
//     ss << "  [10-13] nav_x:    " << std::dec << read_float(10) << " (bytes: ";
//     for (int i = 10; i <= 13; ++i) ss << std::hex << std::setw(2) << (int)data[i] << " ";
//     ss << ")\n";
    
//     ss << "  [14-17] nav_y:    " << std::dec << read_float(14) << " (bytes: ";
//     for (int i = 14; i <= 17; ++i) ss << std::hex << std::setw(2) << (int)data[i] << " ";
//     ss << ")\n";
    
//     ss << "  [18-21] nav_z:    " << std::dec << read_float(18) << " (bytes: ";
//     for (int i = 18; i <= 21; ++i) ss << std::hex << std::setw(2) << (int)data[i] << " ";
//     ss << ")\n";
    
//     ss << "  [22-23] checksum: 0x" << std::hex 
//        << (int)data[23] << (int)data[22] << "\n";
    
//     return ss.str();
//   }
  
//   // 对比两个包是否相同
//   static std::string comparePackets(const SendPacket& a, const SendPacket& b)
//   {
//     std::stringstream ss;
//     ss << "========== Packet Compare ==========\n";
    
//     bool same = true;
    
//     if (a.header != b.header) {
//       ss << "[DIFF] header: " << (int)a.header << " vs " << (int)b.header << "\n";
//       same = false;
//     }
//     if (a.control_byte != b.control_byte) {
//       ss << "[DIFF] control_byte: 0x" << std::hex 
//          << (int)a.control_byte << " vs 0x" << (int)b.control_byte << "\n";
//       same = false;
//     }
//     if (a.pitch != b.pitch) {
//       ss << std::dec << "[DIFF] pitch: " << a.pitch << " vs " << b.pitch << "\n";
//       same = false;
//     }
//     if (a.yaw != b.yaw) {
//       ss << "[DIFF] yaw: " << a.yaw << " vs " << b.yaw << "\n";
//       same = false;
//     }
//     if (a.nav_x != b.nav_x) {
//       ss << "[DIFF] nav_x: " << a.nav_x << " vs " << b.nav_x << "\n";
//       same = false;
//     }
//     if (a.nav_y != b.nav_y) {
//       ss << "[DIFF] nav_y: " << a.nav_y << " vs " << b.nav_y << "\n";
//       same = false;
//     }
//     if (a.nav_z != b.nav_z) {
//       ss << "[DIFF] nav_z: " << a.nav_z << " vs " << b.nav_z << "\n";
//       same = false;
//     }
//     if (a.checksum != b.checksum) {
//       ss << "[DIFF] checksum: " << a.checksum << " vs " << b.checksum << "\n";
//       same = false;
//     }
    
//     if (same) {
//       ss << "[OK] All fields identical\n";
//     }
    
//     return ss.str();
//   }

// private:
//   static const char* stateToString(uint8_t s)
//   {
//     switch (s) {
//       case 0: return "UNTRACKING(未跟踪)";
//       case 1: return "TRACKING_AIM(跟踪瞄准)";
//       case 2: return "TRACKING_BUFF(跟踪大符)";
//       default: return "UNKNOWN(未知)";
//     }
//   }
  
//   static const char* poseToString(uint8_t p)
//   {
//     switch (p) {
//       case 0: return "ATTACK(攻击)";
//       case 1: return "DEFENSE(防御)";
//       case 2: return "MOVE(移动)";
//       default: return "UNKNOWN(未知)";
//     }
//   }
// };

// }  // namespace rm_serial_driver

// #endif  // RM_SERIAL_DRIVER__DEBUG_HPP_