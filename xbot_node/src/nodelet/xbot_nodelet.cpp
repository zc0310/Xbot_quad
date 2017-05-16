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
 * @file /xbot_node/src/nodelet/xbot_nodelet.cpp
 *
 * @brief Implementation for the ROS Xbot nodelet
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "xbot_node/xbot_ros.hpp"


namespace xbot //xbot命名空间
{

class XbotNodelet : public nodelet::Nodelet //定义节点管理器类 ， 公有继承Nodelet 
{
public:
  XbotNodelet() : shutdown_requested_(false) {}; //构造函数
  ~XbotNodelet()    //析构函数
  {
    NODELET_DEBUG_STREAM("Xbot : waiting for update thread to finish.");//线程
    shutdown_requested_ = true;
    update_thread_.join();
  }
  virtual void onInit() //相当于main函数
  {
    NODELET_DEBUG_STREAM("Xbot : initialising nodelet..."); //初始化节点管理器
    std::string nodelet_name = this->getName(); //this是指向XbotNodelet的指针（指向类本身）
    xbot_.reset(new XbotRos(nodelet_name)); //xbot:shared ptr  动态分配内存,返回XbotRos类型的指针 用name初始化（构造函数）
    // if there are latency issues with callbacks, we might want to move to process callbacks
    // in multiple threads (use MTPrivateNodeHandle)
    //如果有回调的延迟问题，我们可能移动到多线程中的进程回调
    if (xbot_->init(this->getPrivateNodeHandle()))
    {
      update_thread_.start(&XbotNodelet::update, *this);
      NODELET_INFO_STREAM("Xbot : initialised.");
    }
    else
    {
      NODELET_ERROR_STREAM("Xbot : could not initialise! Please restart.");
    }
  }
private:  //私有成员
  void update()
  {
    ros::Rate spin_rate(10);
    while (!shutdown_requested_ && ros::ok() && xbot_->update())
    {
      spin_rate.sleep();
    }
  }

  boost::shared_ptr<XbotRos> xbot_; //共享指针 xbot_为XbotRos类
  ecl::Thread update_thread_;
  bool shutdown_requested_;
};

} // namespace xbot

PLUGINLIB_EXPORT_CLASS(xbot::XbotNodelet, nodelet::Nodelet);
