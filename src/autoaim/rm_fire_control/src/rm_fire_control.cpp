#include "rm_fire_control/rm_fire_control.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "auto_aim_interfaces/msg/target.hpp"
#include "math.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace rm_fire_control
{

namespace
{
struct ArmorCandidate
{
  int index = -1;
  double armor_yaw = 0.0;
  double point_yaw = 0.0;
  double score = 0.0;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};
}

double FireController::NormalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double FireController::ShortestAngularDistance(double from, double to) const
{
  return NormalizeAngle(to - from);
}

double FireController::EstimateFlyTime(double x, double y, double z) const
{
  const double target_distance = std::sqrt(x * x + y * y + z * z);
  const double safe_shoot_speed = std::max(Shoot_Speed, 1e-3);
  return target_distance / safe_shoot_speed;
}

void FireController::ApplyAngleStability(
  double raw_pitch,
  double raw_yaw,
  double dt,
  double & stable_pitch,
  double & stable_yaw)
{
  if (dt <= 0.0 || dt > 0.2) {
    dt = 1.0 / 60.0;  /* 时间戳异常时使用默认帧周期 */
  }

  raw_yaw = NormalizeAngle(raw_yaw);

  if (!initialized) {
    stable_pitch = raw_pitch;
    stable_yaw = raw_yaw;
    pre_aim_pitch = stable_pitch;
    pre_aim_yaw = stable_yaw;
    initialized = true;
    return;
  }

  const double max_pitch_step = std::max(Pitch_Max_Change_Speed * dt, 1e-3);
  const double max_yaw_step = std::max(Yaw_Max_Change_Speed * dt, 1e-3);

  double pitch_diff = raw_pitch - pre_aim_pitch;
  double yaw_diff = ShortestAngularDistance(pre_aim_yaw, raw_yaw);

  pitch_diff = std::max(-max_pitch_step, std::min(max_pitch_step, pitch_diff));
  yaw_diff = std::max(-max_yaw_step, std::min(max_yaw_step, yaw_diff));

  stable_pitch = pre_aim_pitch + pitch_diff;
  stable_yaw = NormalizeAngle(pre_aim_yaw + yaw_diff);

  pre_aim_pitch = stable_pitch;
  pre_aim_yaw = stable_yaw;
}

void FireController::ParamInit(void)
{
  Gravity = declare_parameter("Gravity", 9.78);
  Z_Shifting = declare_parameter("Z_Shifting", 0.05);
  Shoot_Speed = declare_parameter<double>("Shoot_Speed", 25.0);
  Communicate_delay = declare_parameter("Communicate_delay", 0.008);
  Ignore_R_Diff = declare_parameter("Ignore_R_Diff", true);
  Ignore_Z_Diff = declare_parameter("Ignore_Z_Diff", true);
  Large_Armor_Width = declare_parameter("Large_Armor_Width", 0.23);
  Small_Armor_Width = declare_parameter("Small_Armor_Width", 0.14);
  Yaw_Res_Speed = declare_parameter("Yaw_Res_Speed", 3.14);
  Yaw_Switch_Hysteresis = declare_parameter("Yaw_Switch_Hysteresis", 0.08);
  Yaw_Max_Change_Speed = declare_parameter("Yaw_Max_Change_Speed", 6.0);
  Pitch_Max_Change_Speed = declare_parameter("Pitch_Max_Change_Speed", 3.0);
  Switch_Confirm_Frames = declare_parameter("Switch_Confirm_Frames", 2);
}

void FireController::ParamUpdate(void)
{
  Gravity = get_parameter("Gravity").as_double();
  Z_Shifting = get_parameter("Z_Shifting").as_double();
  Shoot_Speed = get_parameter("Shoot_Speed").as_double();
  Communicate_delay = get_parameter("Communicate_delay").as_double();
  Ignore_R_Diff = get_parameter("Ignore_R_Diff").as_bool();
  Ignore_Z_Diff = get_parameter("Ignore_Z_Diff").as_bool();
  Yaw_Res_Speed = get_parameter("Yaw_Res_Speed").as_double();
  Large_Armor_Width = get_parameter("Large_Armor_Width").as_double();
  Small_Armor_Width = get_parameter("Small_Armor_Width").as_double();
  Yaw_Switch_Hysteresis = get_parameter("Yaw_Switch_Hysteresis").as_double();
  Yaw_Max_Change_Speed = get_parameter("Yaw_Max_Change_Speed").as_double();
  Pitch_Max_Change_Speed = get_parameter("Pitch_Max_Change_Speed").as_double();
  Switch_Confirm_Frames = static_cast<int>(get_parameter("Switch_Confirm_Frames").as_int());

  Yaw_Switch_Hysteresis = std::max(0.0, Yaw_Switch_Hysteresis);
  Yaw_Max_Change_Speed = std::max(0.1, Yaw_Max_Change_Speed);
  Pitch_Max_Change_Speed = std::max(0.1, Pitch_Max_Change_Speed);
  Switch_Confirm_Frames = std::max(1, Switch_Confirm_Frames);
}

void FireController::Current_yaw_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  current_yaw = NormalizeAngle(msg->data);
}

FireController::FireController(const rclcpp::NodeOptions & options)
: Node("fire_controller", options)
{
  RCLCPP_WARN(this->get_logger(), "FireController is running");
  ParamInit();

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  fired_info_pub_ = this->create_publisher<auto_aim_interfaces::msg::FiredInfo>("/fired_info", 10);
  current_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/serial_driver/current_yaw", 10,
    std::bind(&FireController::Current_yaw_callback, this, std::placeholders::_1));
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

  aim_sub_.subscribe(this, "/tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  aim_time_info_sub_.subscribe(this, "/time_info/aim");
  rune_sub_.subscribe(this, "/tracker/rune");
  buff_time_info_sub_.subscribe(this, "/time_info/buff");

  aim_sync_ = std::make_shared<AimSync>(aim_syncpolicy(500), aim_sub_, aim_time_info_sub_);
  aim_sync_->registerCallback(
    std::bind(&FireController::Choose_board, this, std::placeholders::_1, std::placeholders::_2));

  buff_sync_ = std::make_shared<BuffSync>(buff_syncpolicy(1500), rune_sub_, buff_time_info_sub_);

  aiming_point_.header.frame_id = "map";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);
}

void FireController::Trajectory_Solution(void)
{
  const double target_distance = std::sqrt(
    aim_point_x * aim_point_x + aim_point_y * aim_point_y + aim_point_z * aim_point_z);
  const double safe_distance = std::max(target_distance, 1e-6);
  const double safe_shoot_speed = std::max(Shoot_Speed, 1e-3);

  Fly_time = safe_distance / safe_shoot_speed;

  const double gravity_drop = 0.5 * Gravity * Fly_time * Fly_time;
  const double gravity_compensation = std::atan(gravity_drop / safe_distance);

  aim_pitch = -(std::atan((aim_point_z + Z_Shifting) / safe_distance) + gravity_compensation);
  aim_yaw = NormalizeAngle(std::atan2(aim_point_y, aim_point_x));
}

void FireController::Choose_board(
  const auto_aim_interfaces::msg::Target::ConstSharedPtr msg,
  const auto_aim_interfaces::msg::TimeInfo::ConstSharedPtr time_info)
{
  (void)time_info;
  ParamUpdate();

  if (!msg->tracking) {
    pending_armor_index_ = -1;
    pending_armor_count_ = 0;
    stable_armor_index_ = -1;
    has_last_target_stamp_ = false;
    auto_fire_flag = false;

    auto_aim_interfaces::msg::FiredInfo fired_info_msg;
    fired_info_msg.state = false;
    fired_info_msg.auto_fire_flag = false;
    fired_info_msg.aim_pitch = pre_aim_pitch;
    fired_info_msg.aim_yaw = pre_aim_yaw;
    fired_info_pub_->publish(fired_info_msg);
    return;
  }

  const double dt = has_last_target_stamp_ ?
    std::max((msg->header.stamp - last_target_stamp_).seconds(), 0.0) :
    0.0;
  last_target_stamp_ = msg->header.stamp;
  has_last_target_stamp_ = true;

  const double predicted_fly_time =
    EstimateFlyTime(msg->position.x, msg->position.y, msg->position.z);
  /* 先估一次飞行时间，避免首帧直接使用历史 Fly_time */
  time_delay = Communicate_delay + predicted_fly_time;

  const float aim_x = msg->position.x + static_cast<float>(time_delay * msg->velocity.x);
  const float aim_y = msg->position.y + static_cast<float>(time_delay * msg->velocity.y);
  const float aim_z = msg->position.z + static_cast<float>(time_delay * msg->velocity.z);
  const double predict_yaw = msg->yaw + time_delay * msg->v_yaw;

  if (msg->armors_num <= 0) {
    return;
  }

  const double diff_angle = 2.0 * M_PI / static_cast<double>(msg->armors_num);
  int visible_index = -1;
  double visible_armor_yaw = 0.0;
  double min_visible_abs = std::numeric_limits<double>::max();

  for (int i = 0; i < msg->armors_num; ++i) {
    const double candidate_yaw = NormalizeAngle(predict_yaw + i * diff_angle);
    const double candidate_abs = std::fabs(candidate_yaw);
    if (candidate_abs < diff_angle / 2.0 && candidate_abs < min_visible_abs) {
      min_visible_abs = candidate_abs;
      visible_index = i;
      visible_armor_yaw = candidate_yaw;
    }
  }

  if (visible_index < 0) {
    return;
  }

  const int rotation_dir = (msg->v_yaw >= 0.0) ? 1 : -1;
  const int next_index = (visible_index + rotation_dir + msg->armors_num) % msg->armors_num;

  auto build_candidate = [&](int armor_index, double armor_yaw) {
    ArmorCandidate candidate;
    candidate.index = armor_index;
    candidate.armor_yaw = NormalizeAngle(armor_yaw);

    bool use_first_pair = true;
    if (msg->armors_num == 4) {
      use_first_pair = (armor_index % 2 == 0);
    }

    float radius = static_cast<float>(msg->radius_1);
    float armor_z_value = aim_z;
    if (msg->armors_num == 4) {
      if (!Ignore_R_Diff) {
        radius = use_first_pair ? static_cast<float>(msg->radius_1) :
          static_cast<float>(msg->radius_2);
      }
      if (!Ignore_Z_Diff) {
        armor_z_value = use_first_pair ? aim_z : aim_z + static_cast<float>(msg->dz);
      }
    }

    candidate.x = aim_x - radius * std::cos(candidate.armor_yaw);
    candidate.y = aim_y - radius * std::sin(candidate.armor_yaw);
    candidate.z = armor_z_value;
    candidate.point_yaw = NormalizeAngle(std::atan2(candidate.y, candidate.x));
    candidate.score = std::fabs(ShortestAngularDistance(current_yaw, candidate.point_yaw));
    return candidate;
  };

  const ArmorCandidate current_candidate = build_candidate(visible_index, visible_armor_yaw);
  const ArmorCandidate next_candidate =
    build_candidate(next_index, visible_armor_yaw + rotation_dir * diff_angle);

  const ArmorCandidate *best_candidate = &current_candidate;
  if (next_candidate.score < current_candidate.score) {
    best_candidate = &next_candidate;
  }

  if (stable_armor_index_ < 0) {
    stable_armor_index_ = best_candidate->index;
  } else if (best_candidate->index != stable_armor_index_) {
    const ArmorCandidate *stable_candidate =
      (stable_armor_index_ == current_candidate.index) ? &current_candidate :
      (stable_armor_index_ == next_candidate.index) ? &next_candidate : nullptr;

    const bool has_clear_advantage =
      (stable_candidate == nullptr) ||
      (best_candidate->score + Yaw_Switch_Hysteresis < stable_candidate->score);

    if (has_clear_advantage) {
      if (pending_armor_index_ != best_candidate->index) {
        pending_armor_index_ = best_candidate->index;
        pending_armor_count_ = 1;
      } else {
        ++pending_armor_count_;
      }

      if (pending_armor_count_ >= Switch_Confirm_Frames) {
        stable_armor_index_ = best_candidate->index;
        pending_armor_index_ = -1;
        pending_armor_count_ = 0;
      }
    } else {
      pending_armor_index_ = -1;
      pending_armor_count_ = 0;
    }
  } else {
    pending_armor_index_ = -1;
    pending_armor_count_ = 0;
  }

  const ArmorCandidate *selected_candidate =
    (stable_armor_index_ == next_candidate.index) ? &next_candidate : &current_candidate;

  aim_point_x = selected_candidate->x;
  aim_point_y = selected_candidate->y;
  aim_point_z = selected_candidate->z;
  aim_point_yaw = static_cast<float>(selected_candidate->armor_yaw);

  Trajectory_Solution();

  double stable_pitch = 0.0;
  double stable_yaw = 0.0;
  ApplyAngleStability(aim_pitch, aim_yaw, dt, stable_pitch, stable_yaw);
  aim_pitch = stable_pitch;
  aim_yaw = stable_yaw;

  const float armor_w = (msg->id == "1") ?
    static_cast<float>(Large_Armor_Width) :
    static_cast<float>(Small_Armor_Width);
  const double center_yaw = NormalizeAngle(std::atan2(aim_point_y, aim_point_x));
  const double half_width_yaw =
    std::fabs(std::atan2(0.5 * armor_w, std::max(
      std::sqrt(aim_point_x * aim_point_x + aim_point_y * aim_point_y), 1e-6f)));

  auto_fire_flag = std::fabs(ShortestAngularDistance(current_yaw, center_yaw)) <= half_width_yaw;

  auto_aim_interfaces::msg::FiredInfo fired_info_msg;
  fired_info_msg.state = true;
  fired_info_msg.auto_fire_flag = auto_fire_flag;
  fired_info_msg.aim_pitch = aim_pitch;
  fired_info_msg.aim_yaw = aim_yaw;
  fired_info_pub_->publish(fired_info_msg);

  aiming_point_.header.stamp = this->now();
  aiming_point_.pose.position.x = aim_point_x;
  aiming_point_.pose.position.y = aim_point_y;
  aiming_point_.pose.position.z = aim_point_z;
  marker_pub_->publish(aiming_point_);

  std_msgs::msg::Float64 latency;
  latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
  RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
  latency_pub_->publish(latency);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(rm_fire_control::FireController)



// #include "rm_fire_control/rm_fire_control.hpp"
// #include "math.h"
// #include <iostream>
// #include <rclcpp/rclcpp.hpp>
// #include "auto_aim_interfaces/msg/target.hpp"


// namespace rm_fire_control
// {


// void FireController::ParamInit(void) 
// {
//   Gravity = declare_parameter("Gravity", 9.78);
//   Z_Shifting = declare_parameter("Z_Shifting", 0.05);
//   Shoot_Speed = declare_parameter<double>("Shoot_Speed", 25);
//   Communicate_delay = declare_parameter("Communicate_delay", 0.008);
//   Ignore_R_Diff = declare_parameter("Ignore_R_Diff", true);
//   Ignore_Z_Diff = declare_parameter("Ignore_Z_Diff", true);
//   Large_Armor_Width = declare_parameter("Large_Armor_Width", 0.23);
//   Small_Armor_Width = declare_parameter("Small_Armor_Width", 0.14);
//   Yaw_Res_Speed = declare_parameter("Yaw_Res_Speed", 3.14);
// }

// void FireController::ParamUpdate(void)
// {
//   Gravity = get_parameter("Gravity").as_double();
//   Z_Shifting = get_parameter("Z_Shifting").as_double();
//   Shoot_Speed = get_parameter("Shoot_Speed").as_double();
//   Communicate_delay = get_parameter("Communicate_delay").as_double();
//   Ignore_R_Diff = get_parameter("Ignore_R_Diff").as_bool();
//   Ignore_Z_Diff = get_parameter("Ignore_Z_Diff").as_bool();
//   Yaw_Res_Speed = get_parameter("Yaw_Res_Speed").as_double();
//   Small_Armor_Width = get_parameter("Small_Armor_Width").as_double();
//   Large_Armor_Width = get_parameter("Large_Armor_Width").as_double();
// }

// void FireController::Current_yaw_callback(const std_msgs::msg::Float32::SharedPtr msg)
// {
//   current_yaw = msg->data;
//   ParamUpdate();
// }


// FireController::FireController(const rclcpp::NodeOptions & options)
// : Node("fire_controller", options)
// {
//   RCLCPP_WARN(this->get_logger(), "FireController is running");
//   ParamInit();
//   //create publisher
//   marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
//   fired_info_pub_ = this->create_publisher<auto_aim_interfaces::msg::FiredInfo>("/fired_info", 10);
//   current_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>("/serial_driver/current_yaw", 10, std::bind(&FireController::Current_yaw_callback, this, std::placeholders::_1));
//   latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

//   //create subscriber
//   aim_sub_.subscribe(this, "/tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());
//   aim_time_info_sub_.subscribe(this, "/time_info/aim");
//   rune_sub_.subscribe(this, "/tracker/rune");
//   buff_time_info_sub_.subscribe(this, "/time_info/buff");

//   aim_sync_ = std::make_unique<AimSync>(aim_syncpolicy(500), aim_sub_, aim_time_info_sub_);
//   aim_sync_->registerCallback(
//     std::bind(&FireController::Choose_board, this, std::placeholders::_1, std::placeholders::_2));
  
//   buff_sync_ = std::make_unique<BuffSync>(buff_syncpolicy(1500), rune_sub_, buff_time_info_sub_);
//   // buff_sync_->registerCallback(
//   //   std::bind(&FireController::Choose_board, this, std::placeholders::_1, std::placeholders::_2));
  
//   aiming_point_.header.frame_id = "map";
//   aiming_point_.ns = "aiming_point";
//   aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
//   aiming_point_.action = visualization_msgs::msg::Marker::ADD;
//   aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
//   aiming_point_.color.r = 1.0;
//   aiming_point_.color.g = 1.0;
//   aiming_point_.color.b = 1.0;
//   aiming_point_.color.a = 1.0;
//   aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

// }



// void FireController::Trajectory_Solution()	//忽略空气阻力的弹道解算
// {
// 	//计算弹丸飞行时间
// 	float Target_distance = sqrt((aim_point_x) * (aim_point_x) + (aim_point_y) * (aim_point_y) + (aim_point_z) * (aim_point_z));	//飞行距离
// 	Fly_time = Target_distance / Shoot_Speed;
	
// 	//重力补偿
// 	float temp_x = 0.5 * Gravity * (Fly_time/2) * (Fly_time/2);	//通过飞行时间计算下落距离
// 	double Gravity_Compensation = atan(temp_x / (Target_distance/2));	//由下落距离求得重力补偿角

// 	//正负号记得看情况
// 	// aim_pitch = (double)(atan((aim_point_z+Z_Shifting)/Target_distance))*180/M_PI + Gravity_Compensation*180/M_PI;
// 	// aim_yaw = (double)(atan2(aim_point_y, aim_point_x)) * 180/M_PI;
//   aim_pitch = -((double)(atan((aim_point_z+Z_Shifting)/Target_distance)) + Gravity_Compensation);
// 	aim_yaw = (double)(atan2(aim_point_y, aim_point_x));
// }




// void FireController::Choose_board(  
//   const auto_aim_interfaces::msg::Target::ConstSharedPtr msg,
//   const auto_aim_interfaces::msg::TimeInfo::ConstSharedPtr time_info)
// {
//   // RCLCPP_INFO(this->get_logger(), "-----------%d", msg->tracking);
//   if(msg->tracking)
//   {
//     // ParamUpdate();
//     float armor_x = 0.f, armor_y = 0.f, armor_z = 0.f;	//初始化装甲板坐标
//     // float allow_fire_ang_max = 0.f, allow_fire_ang_min = 0.f;	//初始化开火角度区间 
//     //计算总延时
//     float time_delay = Communicate_delay + Fly_time;
    
//     /*	 状态预测		*/
//     float aim_x = msg->position.x + time_delay * msg->velocity.x;
//     float aim_y = msg->position.y + time_delay * msg->velocity.y;
//     float aim_z = msg->position.z + time_delay * msg->velocity.z;
//     float predict_yaw = msg->yaw + time_delay * msg->v_yaw;



//     /*   选板		*/
//     uint8_t use_1 = 1;	//判断是否为前目标板，默认为1，下一块则为0
//     float diff_angle = 2 * M_PI / msg->armors_num;	//计算两块装甲板角度差
//     uint8_t armor_choose = 0;
//     int i = 0;
//     while (abs(i) < msg->armors_num) {	//逆时针遍历装甲板
//       float armor_yaw = predict_yaw + i * diff_angle;	//推理四块装甲板的角度
//       if (abs(armor_yaw) < diff_angle / 2) {								
//         armor_choose = 1;	//若装甲板yaw轴相对角度<45度，进行选择（四块装甲板只选一其中块）

//         // 四块装甲板对应两个r两个z
//         float r = msg->radius_1;	//默认忽略r偏差
//         armor_z = aim_z;						//默认忽略z偏差
        
//         /*是否修正z和r上偏差*/
//         if (msg->armors_num == 4) {					
//           if(!Ignore_R_Diff){	r = use_1 ? msg->radius_1 : msg->radius_2;}				//启用r偏差
//           if(!Ignore_Z_Diff){	armor_z = use_1 ? aim_z : (aim_z + msg->dz);}	//启用z偏移
//         }

//         // 计算当前装甲板对应预瞄点
//         armor_x = aim_x - r * cos(armor_yaw);		
//         armor_y = aim_y - r * sin(armor_yaw);

//         // 计算下一块装甲板的数据
//         float next_r = msg->radius_1;//默认忽略r偏差
//         float armor_z_next = aim_z;			//默认忽略z偏差
//         /*是否修正z和r上偏差*/
//         if (msg->armors_num == 4) {
//           if(!Ignore_R_Diff){	next_r = !use_1 ? msg->radius_1 : msg->radius_2;}				//启用r偏差
//           if(!Ignore_Z_Diff){	armor_z_next = !use_1 ? aim_z : (aim_z + msg->dz);}		//启用z偏移
//         }
//         // 根据转向判断下一个装甲板的位置角
//         float next_armor_yaw =
//             armor_yaw +
//             signbit(msg->v_yaw) * diff_angle;  // 下一装甲板yaw等于当前yaw-转速（方向）*90度
        
//         //同理计算下一个装甲板的预瞄点
//         float armor_x_next = aim_x - next_r * cos(next_armor_yaw);
//         float armor_y_next = aim_y - next_r * sin(next_armor_yaw);
        
//         //选板部分

//         //计算甩头期间旋转角度
//         float yaw_motor_delta = fabs(atan2(armor_y, armor_x)-current_yaw)/Yaw_Res_Speed * msg->v_yaw / 2;
//         float yaw_notor_next_delta = fabs(atan2(armor_y_next, armor_x_next)-current_yaw)/Yaw_Res_Speed * msg->v_yaw / 2;

//         //切板决策 不切板偏转角度（中心夹角）小于切板夹角时放弃切板
//         RCLCPP_INFO(this->get_logger(),"current board angle:%lf   next board angle:%lf",armor_yaw,next_armor_yaw);    
//         RCLCPP_INFO(this->get_logger(),"current delta:%lf   next delta:%lf",yaw_motor_delta,yaw_notor_next_delta);    
           
//         if (fabs(armor_yaw + yaw_motor_delta) < fabs(next_armor_yaw + yaw_notor_next_delta)) {		
//           aim_point_x = armor_x;
//           aim_point_y = armor_y;
//           aim_point_z = armor_z;
//           aim_point_yaw = armor_yaw;
//           RCLCPP_INFO(this->get_logger(),"use_board: current board is used!");
//         } else {	//切板
//           aim_point_x = armor_x_next;
//           aim_point_y = armor_y_next;
//           aim_point_z = armor_z_next;
//           aim_point_yaw = next_armor_yaw;
//           RCLCPP_INFO(this->get_logger(),"use_board: next board is used!");
//         }

//         /*  自动开火  */
//         float armor_w;
//         if (msg->id == "1")
//           armor_w = Large_Armor_Width;	//大装甲板宽度
//         else
//           armor_w = Small_Armor_Width;	//小装甲板宽度
//         float ax = aim_point_x - 0.5f * armor_w * sin(aim_point_yaw);
//         float ay = aim_point_y + 0.5f * armor_w * cos(aim_point_yaw);
//         float bx = aim_point_x + 0.5f * armor_w * sin(aim_point_yaw);
//         float by = aim_point_y - 0.5f * armor_w * cos(aim_point_yaw);
//         float angle_a = atan2(ay, ax);  //最大角
//         float angle_b = atan2(by, bx);  //最小角

//         Trajectory_Solution();	//弹道解算
        
//         if(armor_choose ==  1){
//           auto_fire_flag = (current_yaw < angle_a)&&(current_yaw > angle_b);
//         }
//         auto_aim_interfaces::msg::FiredInfo fired_info_msg;
//         fired_info_msg.state = msg->tracking;
//         fired_info_msg.auto_fire_flag = auto_fire_flag;
//         fired_info_msg.aim_pitch = aim_pitch;
//         fired_info_msg.aim_yaw = aim_yaw;
//         fired_info_pub_->publish(fired_info_msg);
//         pre_aim_pitch = aim_pitch;
//         pre_aim_yaw = aim_yaw;

//         aiming_point_.header.stamp = this->now();
//         aiming_point_.pose.position.x = aim_point_x;
//         aiming_point_.pose.position.y = aim_point_y;
//         aiming_point_.pose.position.z = aim_point_z;
//         marker_pub_->publish(aiming_point_);

//         std_msgs::msg::Float64 latency;
//         latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
//         RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
//         latency_pub_->publish(latency);

//         break;
//       }
//       else{
//         armor_choose = 0;	//大于45度不选板
//       }
//       use_1 = !use_1;
//       i+=1;
//     }
//   }
//   else{
//     auto_aim_interfaces::msg::FiredInfo fired_info_msg;
//     fired_info_msg.state = msg->tracking;
//     fired_info_msg.auto_fire_flag = 0;
//     fired_info_msg.aim_pitch = pre_aim_pitch;
//     fired_info_msg.aim_yaw = pre_aim_yaw;
//     fired_info_pub_->publish(fired_info_msg);
//   }
// }



// }   // namespace fire_control


// #include "rclcpp_components/register_node_macro.hpp"

// // Register the component with class_loader.
// // This acts as a sort of entry point, allowing the component to be discoverable when its library
// // is being loaded into a running process.
// RCLCPP_COMPONENTS_REGISTER_NODE(rm_fire_control::FireController)

