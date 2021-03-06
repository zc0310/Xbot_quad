/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file /xbot_node/src/node/subscriber_callbacks.cpp
 *
 * @brief Subscriber callbacks for xbot node.
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "../../include/xbot_node/xbot_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void XbotRos::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
{
  if (xbot.isEnabled())
  {
    // For now assuming this is in the robot frame, but probably this
    // should be global frame and require a transform
    //double vx = msg->linear.x;        // in (m/s)
    //double wz = msg->angular.z;       // in (rad/s)
    ROS_ERROR_STREAM("Xbot : velocity command received [" << msg->linear.x << "],[" << msg->linear.y << "],[" << msg->angular.z << "]");
    xbot.setBaseControl(msg->linear.x, msg->linear.y, msg->angular.z);
    odometry.resetTimeout();
  }
  return;
}






/**
 * @brief Play a predefined sound (single sound or sound sequence)
 */

/**
 * @brief Reset the odometry variables.
 */
void XbotRos::subscribeResetOdometry(const std_msgs::EmptyConstPtr /* msg */)
{
  ROS_INFO_STREAM("Xbot : Resetting the odometry. [" << name << "].");
  joint_states.position[0] = 0.0; // wheel_left
  joint_states.velocity[0] = 0.0;
  joint_states.position[1] = 0.0; // wheel_right
  joint_states.velocity[1] = 0.0;
  joint_states.position[2] = 0.0; // wheel_left
  joint_states.velocity[2] = 0.0;
  joint_states.position[3] = 0.0; // wheel_right
  joint_states.velocity[3] = 0.0;
  joint_states.position[4] = 0.0; // wheel_left
  joint_states.velocity[4] = 0.0;
  joint_states.position[5] = 0.0; // wheel_left
  joint_states.velocity[5] = 0.0;
  joint_states.position[6] = 0.0; // wheel_left
  joint_states.velocity[6] = 0.0;
  odometry.resetOdometry();
  xbot.resetOdometry();
  return;
}

void XbotRos::subscribeMotorPower(const xbot_msgs::MotorPowerConstPtr msg)
{
  if (msg->state == xbot_msgs::MotorPower::ON)
  {
    ROS_INFO_STREAM("Xbot : Firing up the motors. [" << name << "]");
    xbot.enable();
    odometry.resetTimeout();
  }
  else if (msg->state == xbot_msgs::MotorPower::OFF)
  {
    xbot.disable();
    ROS_INFO_STREAM("Xbot : Shutting down the motors. [" << name << "]");
    odometry.resetTimeout();
  }
  else
  {
    ROS_ERROR_STREAM("Xbot : Motor power command specifies unknown state '" << (unsigned int)msg->state
                     << "'. [" << name << "]");
  }
}

void XbotRos::subscribeMotorControlCommand(const xbot_msgs::XbotState msg)
{
    xbot.setLiftControl(msg.height_percent);
    xbot.setPlatformCameraControl(msg.platform_degree, msg.camera_degree);
}

void XbotRos::subscribeControllerInfoCommand(const xbot_msgs::ControllerInfoConstPtr msg)
{
  if( msg->p_gain < 0.0f ||  msg->i_gain < 0.0f ||  msg->d_gain < 0.0f) {
    ROS_ERROR_STREAM("Xbot : All controller gains should be positive. [" << name << "]");
    return;
  }
  return;
}

} // namespace xbot
