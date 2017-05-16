/**
 * @file /xbot_driver/include/xbot_driver/modules/diff_drive.hpp
 *
 * @brief Simple module for the diff drive odometry.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef XBOT_DIFF_DRIVE_HPP_
#define XBOT_DIFF_DRIVE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <climits>
#include <stdint.h>
#include <ecl/mobile_robot.hpp>
#include <ecl/threads/mutex.hpp>
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class xbot_PUBLIC DiffDrive { //差分驱动里程计模型
public:
  DiffDrive();
  const ecl::DifferentialDrive::Kinematics& kinematics() { return diff_drive_kinematics; }
  void update(const unsigned int &time_stamp,   //时间戳
              const uint16_t &encoder_1,
              const uint16_t &encoder_2,
              const uint16_t &encoder_3,
              const uint16_t &encoder_4,
              ecl::Pose2D<double> &pose_update, //位置更新
              ecl::linear_algebra::Vector3d &pose_update_rates);
  void reset();
  void getWheelJointStates(float &wheel_1_angle, float &wheel_1_angle_rate,
                           float &wheel_2_angle, float &wheel_2_angle_rate,
                           float &wheel_3_angle, float &wheel_3_angle_rate,
                           float &wheel_4_angle, float &wheel_4_angle_rate);
  void setVelocityCommands(const float &vx, const float &vy, const float &wz);
  void velocityCommands(const float &vx, const float &vy, const float &wz);
  void velocityCommands(const std::vector<float> &cmd) { velocityCommands(cmd[0], cmd[1], cmd[2]); }

  /*********************
  ** Command Accessors 指令访问器
  **********************/
  std::vector<float> velocityCommands(); // (speed, radius), in [mm/s] and [mm]
  std::vector<float> pointVelocity() const; // (vx,vy,wz), in [m/s] and [rad/s]

  /*********************
  ** Property Accessors
  **********************/
  float wheel_bias() const { return bias; }  //轮子轴距

private:
  unsigned int last_timestamp;
  float last_velocity_1, last_velocity_2, last_velocity_3, last_velocity_4;
  float last_diff_time;

  unsigned short last_tick_1, last_tick_2, last_tick_3, last_tick_4;
  float last_rad_1, last_rad_2, last_rad_3, last_rad_4;

  //float v, w; // in [m/s] and [rad/s]
  std::vector<float> point_velocity;
  //point_velocity.resize(3);
  // (vx,vy,wz), in [m/s] and [rad/s]
  float angular_velocity; // in [m/s]
  //float linear_velocity;  // in [rad/s]
  float linear_velocity_x;
  float linear_velocity_y;
  float bias; //wheelbase, wheel_to_wheel, in [m]  轴距
  float wheel_radius; // in [m]
  int   imu_heading_offset;
  const float tick_to_rad;

  ecl::DifferentialDrive::Kinematics diff_drive_kinematics;
  ecl::Mutex velocity_mutex, state_mutex;

  // Utility
  short bound(const float &value);
};

} // namespace xbot

#endif /* XBOT_DIFF_DRIVE_HPP_ */
