/**
 * @file /xbot_driver/src/driver/diff_drive.cpp
 *
 * @brief Differential drive abstraction (brought in from ycs).
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/modules/diff_drive.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Implementation
*****************************************************************************/
DiffDrive::DiffDrive() : //构造函数初始化
  last_velocity_1(0.0),
  last_velocity_2(0.0),
  last_velocity_3(0.0),
  last_velocity_4(0.0),
  last_tick_1(0),
  last_tick_2(0),
  last_tick_3(0),
  last_tick_4(0),
  last_rad_1(0.0),
  last_rad_2(0.0),
  last_rad_3(0.0),
  last_rad_4(0.0),
  angular_velocity(0.0), //linear_velocity(0.0), // command velocities, in [mm] and [mm/s]
  linear_velocity_x(0.0),
  linear_velocity_y(0.0),
  point_velocity(3,0.0), // command velocities, in [m/s] and [rad/s]
  bias(0.39), // wheelbase, wheel_to_wheel, in [m]   ?????
  wheel_radius(0.0625), // radius of main wheel, in [m]          ?????
  tick_to_rad(0.00078539815f) //f表示单精度浮点数 编码器每计一次数对应的角度
  //diff_drive_kinematics(bias, wheel_radius)    //传入两参数：轴距和轮半径
 {}

/**
 * @brief Updates the odometry from firmware stamps and encoders.
 *
 * Really horrible - could do with an overhaul.
 *
 * @param time_stamp
 * @param left_encoder
 * @param right_encoder
 * @param pose_update
 * @param pose_update_rates
 */

void DiffDrive::update(const unsigned int &time_stamp,   //该update函数主要目的是更新位置信息
                      // const uint16_t &left_encoder,
                      // const uint16_t &right_encoder,
                       const uint16_t &encoder_1,
                       const uint16_t &encoder_2,
                       const uint16_t &encoder_3,
                       const uint16_t &encoder_4,
                       ecl::Pose2D<double> &pose_update,
                       ecl::linear_algebra::Vector3d &pose_update_rates) {
  state_mutex.lock();
  static bool init_1 = false;
  static bool init_2 = false;
  static bool init_3 = false;
  static bool init_4 = false;

  float diff_ticks_1 = 0.0f;
  float diff_ticks_2 = 0.0f;
  float diff_ticks_3 = 0.0f;
  float diff_ticks_4 = 0.0f;

  unsigned short curr_tick_1 = 0;
  unsigned short curr_tick_2 = 0;
  unsigned short curr_tick_3 = 0;
  unsigned short curr_tick_4 = 0;

  unsigned int curr_timestamp = 0;
  curr_timestamp = time_stamp;

  curr_tick_1 = encoder_1;        //记录轮子1编码器并更新
  if (!init_1)
  {
    last_tick_1 = curr_tick_1;
    init_1 = true;
  }
  diff_ticks_1 = (float)(short)((curr_tick_1 - last_tick_1) & 0xffff);   //编码器1码盘计数(差分),与ffff进行与运算还是它本身
  last_tick_1  = curr_tick_1;
  last_rad_1  += tick_to_rad * diff_ticks_1;        //将左轮角度变化量赋给last_rad_1

  curr_tick_2 = encoder_2;        //记录轮子2编码器并更新
  if (!init_2)
  {
    last_tick_2 = curr_tick_2;
    init_2 = true;
  }
  diff_ticks_2 = (float)(short)((curr_tick_2 - last_tick_2) & 0xffff);
  last_tick_2  = curr_tick_2;
  last_rad_2  += tick_to_rad * diff_ticks_2;

  curr_tick_3 = encoder_3;        //记录轮子3编码器并更新
  if (!init_3)
  {
    last_tick_3 = curr_tick_3;
    init_3 = true;
  }
  diff_ticks_3 = (float)(short)((curr_tick_3 - last_tick_3) & 0xffff);
  last_tick_3  = curr_tick_3;
  last_rad_3  += tick_to_rad * diff_ticks_3;

  curr_tick_4 = encoder_4;        //记录轮子4编码器并更新
  if (!init_4)
  {
    last_tick_4 = curr_tick_4;
    init_4 = true;
  }
  diff_ticks_4 = (float)(short)((curr_tick_4 - last_tick_4) & 0xffff);
  last_tick_4  = curr_tick_4;
  last_rad_4  += tick_to_rad * diff_ticks_4;

  // TODO this line and the last statements are really ugly; refactor, put in another place
  //pose_update = diff_drive_kinematics.forward((double)(tick_to_rad * diff_ticks_1), (double)(tick_to_rad * diff_ticks_2));

//  double dx = 0.707106781 * wheel_radius * ((double)(tick_to_rad * diff_ticks_3) + (double)(tick_to_rad * diff_ticks_4));
//  double dy = 0.707106781 * wheel_radius * ((double)(tick_to_rad * diff_ticks_3) - (double)(tick_to_rad * diff_ticks_2));
//  double domega = wheel_radius * ((double)(tick_to_rad * diff_ticks_3) - (double)(tick_to_rad * diff_ticks_1)) / bias;
//  pose_update.translation(dx,dy);
//  pose_update.rotation(domega);

  //根据四个码盘数据计算(注意方向和系数)
  double dx = 0.353553391 * wheel_radius * ((double)(tick_to_rad * diff_ticks_3) + (double)(tick_to_rad * diff_ticks_4) +
               (double)(tick_to_rad * diff_ticks_1) + (double)(tick_to_rad * diff_ticks_2));
  double dy = 0.353553391 * wheel_radius * (-(double)(tick_to_rad * diff_ticks_3) + (double)(tick_to_rad * diff_ticks_4) -
               (double)(tick_to_rad * diff_ticks_1) + (double)(tick_to_rad * diff_ticks_2));
  double domega = wheel_radius * (-(double)(tick_to_rad * diff_ticks_3) + (double)(tick_to_rad * diff_ticks_4) +
               (double)(tick_to_rad * diff_ticks_1) - (double)(tick_to_rad * diff_ticks_2)) / (2 * bias);
  pose_update.translation(dx,dy);
  pose_update.rotation(domega);

  if (curr_timestamp != last_timestamp) //时间戳发生改变
  {
  //last_diff_time = ((double)(short)((curr_timestamp - last_timestamp) & 0xffff)) / 1000000.0f;//以us计数
    last_diff_time = 0.02;
    last_timestamp = curr_timestamp;
    last_velocity_1 = (tick_to_rad * diff_ticks_1) / last_diff_time;
    last_velocity_2 = (tick_to_rad * diff_ticks_2) / last_diff_time;
    last_velocity_3 = (tick_to_rad * diff_ticks_3) / last_diff_time;
    last_velocity_4 = (tick_to_rad * diff_ticks_4) / last_diff_time;
  }
  else
  {
    // we need to set the last_velocity_xxx to zero?
  }

  pose_update_rates << pose_update.x() / last_diff_time,  //????
                       pose_update.y() / last_diff_time,
                       pose_update.heading() / last_diff_time;

  std::cout<< "odometry updates successfully" << std::endl;
  state_mutex.unlock();
}

void DiffDrive::reset() {
  state_mutex.lock();
  last_rad_1 = 0.0;
  last_rad_2 = 0.0;
  last_rad_3 = 0.0;
  last_rad_4 = 0.0;

  last_velocity_1 = 0.0;
  last_velocity_2 = 0.0;
  last_velocity_3 = 0.0;
  last_velocity_4 = 0.0;
  state_mutex.unlock();
}

void DiffDrive::getWheelJointStates(float &wheel_1_angle, float &wheel_1_angle_rate,
                                    float &wheel_2_angle, float &wheel_2_angle_rate,
                                    float &wheel_3_angle, float &wheel_3_angle_rate,
                                    float &wheel_4_angle, float &wheel_4_angle_rate)
{
  state_mutex.lock();

  wheel_1_angle = last_rad_1;
  wheel_2_angle = last_rad_2;
  wheel_3_angle = last_rad_3;
  wheel_4_angle = last_rad_4;

  wheel_1_angle_rate = last_velocity_1;
  wheel_2_angle_rate = last_velocity_2;
  wheel_3_angle_rate = last_velocity_3;
  wheel_4_angle_rate = last_velocity_4;
  state_mutex.unlock();
}

void DiffDrive::setVelocityCommands(const float &vx, const float &vy, const float &wz) { //将控制指令放到一个向量
  std::vector<float> cmd_vel;
  cmd_vel.push_back(vx);
  cmd_vel.push_back(vy);
  cmd_vel.push_back(wz);
  point_velocity = cmd_vel;
  //return
}

void DiffDrive::velocityCommands(const float &vx, const float &vy, const float &wz) {
  velocity_mutex.lock();
  linear_velocity_x = vx;
  linear_velocity_y = vy;
  angular_velocity  = wz;
  velocity_mutex.unlock();
    return;
}


std::vector<float> DiffDrive::pointVelocity() const { //返回控制指令
  return point_velocity;
}

std::vector<float> DiffDrive::velocityCommands() {
  velocity_mutex.lock();
  std::vector<float> cmd(3);
  cmd[0] = linear_velocity_x;  // In [m/s]
  cmd[1] = linear_velocity_y;
  cmd[2] = angular_velocity*180/ecl::pi; // In [degree/s] 转换成角度
  velocity_mutex.unlock();
  return cmd;
}

short DiffDrive::bound(const float &value) {
  if (value > static_cast<float>(SHRT_MAX)) return SHRT_MAX;
  if (value < static_cast<float>(SHRT_MIN)) return SHRT_MIN;
  return static_cast<short>(value);
}

} // namespace xbot
