/**
 * @file src/driver/xbot.cpp
 *
 * @brief Implementation for the xbot device driver.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <fstream>
#include <ecl/math.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/time/sleep.hpp>
#include <ecl/converters.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/time/timestamp.hpp>
#include <stdexcept>
#include "../../include/xbot_driver/xbot.hpp"
#include "../../include/xbot_driver/packet_handler/payload_headers.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/
//xbot设备驱动的具体实现
namespace xbot
{

/*****************************************************************************
 ** Implementation [PacketFinder]
 *****************************************************************************/

  bool PacketFinder::checkSum()   //计算最后一位异或位的函数
  {
    unsigned int packet_size(buffer.size());
    unsigned char cs(0);
    for (unsigned int i = 2; i < packet_size; i++)
    {
      cs ^= buffer[i]; //位异或运算:相同为0,不同为1;与0相异或，保留原值
    }
    return cs ? false : true;
  }

/*****************************************************************************
 ** Implementation [Initialisation]
 *****************************************************************************/

  Xbot::Xbot() : //构造函数 根据C++的规则，const类型和引用不可以被赋值，只能被初始化
    //构造函数后面的冒号就是初始化，而括号里面的等于号并不是初始化，而是变量生成以后的赋值而已（永远都是2个步骤）。
    //C++给类成员初始化的唯一方式就是成员初始化列表，也就是构造函数后面跟冒号的那种形式
      shutdown_requested(false)
      , is_enabled(false)
      , is_connected(false)
      , is_alive(false)
      , heading_offset(0.0)
      , HeightPercent(0)
      , CameraDegree(90)
      , PlatformDegree(90)
{}

/**
 * Shutdown the driver - make sure we wait for the thread to finish.
 */
  Xbot::~Xbot()  //析构函数
  {
    disable();
    shutdown_requested = true; // thread's spin() will catch this and terminate
    thread.join();
    resetXbotState();
    sig_debug.emit("Device: xbot driver terminated.");//输出调试信息
  }

  void Xbot::init(Parameters &parameters) throw (ecl::StandardException) //初始化 throw异常处理
  //Xbot初始化需要传递parameters参数进来       //函数init能够抛出也只能抛出StandardException及它们子类型的异常
  {
    if (!parameters.validate())
    {
      throw ecl::StandardException(LOC, ecl::ConfigurationError, "Xbot's parameter settings did not validate.");
    }
    this->parameters = parameters; //将参数传给xbot类本身的parameters变量
    std::string sigslots_namespace = parameters.sigslots_namespace;
    event_manager.init(sigslots_namespace);

    // connect signals
    sig_controller_info.connect(sigslots_namespace + std::string("/controller_info"));
    sig_stream_data.connect(sigslots_namespace + std::string("/stream_data"));
    sig_raw_data_command.connect(sigslots_namespace + std::string("/raw_data_command"));
    sig_raw_data_stream.connect(sigslots_namespace + std::string("/raw_data_stream"));
    sig_raw_control_command.connect(sigslots_namespace + std::string("/raw_control_command"));
    //sig_serial_timeout.connect(sigslots_namespace+std::string("/serial_timeout"));

    sig_debug.connect(sigslots_namespace + std::string("/ros_debug"));
    sig_info.connect(sigslots_namespace + std::string("/ros_info"));
    sig_warn.connect(sigslots_namespace + std::string("/ros_warn"));
    sig_error.connect(sigslots_namespace + std::string("/ros_error"));
    sig_named.connect(sigslots_namespace + std::string("/ros_named"));

    try {
      serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
      // this will throw exceptions - NotFoundError, OpenError
      is_connected = true;
      serial.block(4000); // blocks by default, but just to be clear!
    }
    catch (const ecl::StandardException &e)
    {
      if (e.flag() == ecl::NotFoundError) {
        sig_warn.emit("device does not (yet) available, is the usb connected?."); // not a failure mode.
      } else {
        throw ecl::StandardException(LOC, e);
      }
    }

    ecl::PushAndPop<unsigned char> stx(2, 0);//定义两个PushandPop类的对象,，一个头一个尾
    ecl::PushAndPop<unsigned char> etx(1);
    stx.push_back(0xaa);
    stx.push_back(0x55);
    packet_finder.configure(sigslots_namespace, stx, etx, 1, 256, 1, true); //数据包
    acceleration_limiter.init(parameters.enable_acceleration_limiter); //加速度限制器初始化
    //增加横向加速度的限制
    // in case the user changed these from the defaults
    Battery::capacity = parameters.battery_capacity;
    Battery::low = parameters.battery_low;
    Battery::dangerous = parameters.battery_dangerous;


    thread.start(&Xbot::spin, *this);
  }

/*****************************************************************************
 ** Implementation [Runtime]
 *****************************************************************************/
/**
 * Usually you should call the getXXX functions from within slot callbacks
 * connected to this driver's signals. This ensures that data is not
 * overwritten inbetween getXXX calls as it all happens in the serial device's
 * reading thread (aye, convoluted - apologies for the multiple robot and multiple
 * developer adhoc hacking over 4-5 years for hasty demos on pre-xbot robots.
 * This has generated such wonderful spaghetti ;).
 *
 * If instead you just want to poll xbot, then you should lock and unlock
 * the data access around any getXXX calls.
 */
  void Xbot::lockDataAccess() {
    data_mutex.lock();
  }

/**
 * Unlock a previously locked data access privilege.
 * @sa lockDataAccess()
 */
  void Xbot::unlockDataAccess() {
    data_mutex.unlock();
  }

/**
 * @brief Performs a scan looking for incoming data packets.
 * 执行扫描寻找传入的数据包
 * Sits on the device waiting for incoming and then parses it, and signals
 * that an update has occured.
 * 在设备上等待传入，然后解析它，并发出更新发生的信号。
 * Or, if in simulation, just loopsback the motor devices.
 */

  void Xbot::spin()
  {
    ecl::TimeStamp last_signal_time;
    ecl::Duration timeout(0.1);
    unsigned char buf[256];


  //  int buf_size=0;


    /*********************
     ** Simulation Params
     **********************/

    while (!shutdown_requested)
    {
      /*********************
       ** Checking Connection
       **********************/
      if ( !serial.open() ) {
        try {
          // this will throw exceptions - NotFoundError is the important one, handle it
          serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
          sig_info.emit("device is connected.");
          is_connected = true;
          serial.block(4000); // blocks by default, but just to be clear!
          event_manager.update(is_connected, is_alive);
        }
        catch (const ecl::StandardException &e)
        {
          // windows throws OpenError if not connected
          if (e.flag() == ecl::NotFoundError) {
            sig_info.emit("device does not (yet) available on this port, waiting...");
          } else if (e.flag() == ecl::OpenError) {
            sig_info.emit("device failed to open, waiting... [" + std::string(e.what()) + "]");
          } else {
            // This is bad - some unknown error we're not handling! But at least throw and show what error we came across.
            throw ecl::StandardException(LOC, e);
          }
          ecl::Sleep(5)(); // five seconds
          is_connected = false;
          is_alive = false;
          continue;
        }
      }

      /*********************
       ** Read Incoming 读串口
       **********************/
      int n = serial.read((char*)buf, 1/*packet_finder.numberOfDataToRead()*/);

      if (n == 0)
      {
        if (is_alive && ((ecl::TimeStamp() - last_signal_time) > timeout))
        {
          is_alive = false;
          sig_debug.emit("Timed out while waiting for incoming bytes.");
        }
        event_manager.update(is_connected, is_alive);
        continue;
      }
      else
      {
        std::ostringstream ostream;
        ostream << "xbot_node : serial_read(" << n << ")"
          << ", packet_finder.numberOfDataToRead(" << packet_finder.numberOfDataToRead() << ")";
        //sig_debug.emit(ostream.str());
        //might be useful to send this to a topic if there is subscribers
      }
  //    buf_size+=n;

      bool find_packet = packet_finder.update(buf, n); //将n更新到buf上


      if (find_packet) // this clears packet finder's buffer and transfers important bytes into it
      {//这清除了数据包查找器的缓冲区并将重要字节传输到其中; 对传感器数据进行反序列化，并对控制指令进行打包(序列化)写入串口

        PacketFinder::BufferType local_buffer;
        packet_finder.getBuffer(local_buffer); // get a reference to packet finder's buffer.
        //获取对数据包查找器缓冲区的引用,将buffer的值赋给local_buffer
        sig_raw_data_stream.emit(local_buffer);

        packet_finder.getPayload(data_buffer);// get a reference to packet finder's buffer.
        //将缓冲区的实际数据内容推入data_buffer(抛去头、尾、长度、校验)

        lockDataAccess();  //锁止(开始进行数据处理)
        std::cout<<"data_buffer.size:"<<data_buffer.size()<<std::endl;

        while (data_buffer.size() >0)
        {

  /*
   * std::cout << "header_id: " << (unsigned int)data_buffer[0] << " | "<<std::endl;
            std::cout << "length: " << (unsigned int)data_buffer[1] << " | ";
            std::cout << "remains: " << data_buffer.size() << " | ";
            std::cout << "local_buffer: " << local_buffer.size() << " | ";
            std::cout << std::endl;*/
          switch (data_buffer[0])
          {
            // these come with the streamed feedback
              case Header::CoreSensors: //直接用标识符进行判断
                  std::cout<<"come into core sensors"<<std::endl;
                  if( !core_sensors.deserialise(data_buffer) ) //反序列化(反解指令得到具体的core传感器数据)
                  {
                    fixPayload(data_buffer);//将序列中的每一个数据输出
                    break;
                  }
                  sig_stream_data.emit(); //打开通道让数据出去
                  break;
              case Header::ImuSensors:
  //                sig_raw_data_stream.emit(local_buffer);
                  if( !imu_sensors.deserialise(data_buffer) )//反序列化(反解指令得到具体的imu传感器数据)
                  {
                    fixPayload(data_buffer);             //序列中第一个元素head_id是标识符
                      break;
                  }
  //              sig_stream_data.emit();
                  break;

              default: // in the case of unknown or mal-formed sub-payload
                  fixPayload(data_buffer);
                  break;
          }
        }
        std::cout << "-------------------------------------------------" << std::endl;
        unlockDataAccess();

        is_alive = true;//数据流监控标识符
        event_manager.update(is_connected, is_alive);
        last_signal_time.stamp();
  //      sig_stream_data.emit();
        sendBaseControlCommand(); // send the command packet to mainboard;发送基本控制指令(线速度 角速度在此处被转换为十六进制序列)
      }
      else
      {
        // watchdog
        if (is_alive && ((ecl::TimeStamp() - last_signal_time) > timeout))
        {
          is_alive = false;
          // do not call here the event manager update, as it generates a spurious offline state
        }
      }
    }
    sig_error.emit("Driver worker thread shutdown!");
  }

  void Xbot::fixPayload(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (byteStream.size() < 3 ) { /* minimum size of sub-payload is 3; header_id, length, data */
      byteStream.clear();
    } else {
      std::stringstream ostream;
      unsigned int header_id = static_cast<unsigned int>(byteStream.pop_front());//标识符
      unsigned int length = static_cast<unsigned int>(byteStream.pop_front());   //路数
      unsigned int remains = byteStream.size();
      unsigned int to_pop;

      ostream << "[" << header_id << "]";
      ostream << "[" << length << "]";

      ostream << "[";
      ostream << std::setfill('0') << std::uppercase;
      ostream << std::hex << std::setw(2) << header_id << " " << std::dec; //header_id占2个位置
      ostream << std::hex << std::setw(2) << length << " " << std::dec; //length占2个位置

      if (remains < length) to_pop = remains;
      else                  to_pop = length; //最多只输出标识符和实际数据

      for (unsigned int i = 0; i < to_pop; i++ ) {
        unsigned int byte = static_cast<unsigned int>(byteStream.pop_front());
        ostream << std::hex << std::setw(2) << byte << " " << std::dec;//以16进制的形式挨个输出每一数据

      }
      ostream << "]";

    }
  }


/*****************************************************************************
 ** Implementation [Human Friendly Accessors]人性化的访问器
 *****************************************************************************/

  float Xbot::getHeading() const //获取朝向
  {
    ecl::Angle<float>heading; //角度测量(弧度制)
  //    float heading;
    // raw data angles are in tens of a degree, convert to radians.
    heading = (static_cast<float>(imu_sensors.data.yaw) / 10.0)*ecl::pi / 180.0; //朝向计算
    std::cout<<"heading: "<<heading<<" | heading_offset: "<<heading_offset<<std::endl;

    return ecl::wrap_angle(heading - heading_offset);
  }

  /*
   * int Xbot::getDebugSensors() const
  {
      return (static_cast<int>(core_sensors.data.left_encoder)); //获取左编码器
  }*/

  float Xbot::getAngularVelocity() const
  {
    // raw data angles are in hundredths of a degree, convert to radians.
    return (static_cast<float>(imu_sensors.data.yaw) / 10.0) * ecl::pi / 180.0;  //????和上面一样
  }



  void Xbot::resetXbotState()
  {
      setLiftControl(0);
      setPlatformCameraControl(90,90);
  }

  /*****************************************************************************
   ** Implementation [Raw Data Accessors]  原始数据访问器
   *****************************************************************************/

  void Xbot::resetOdometry()
  {
    diff_drive.reset();

    // Issue #274: use current imu reading as zero heading to emulate reseting gyro 使用当前imu读数作为零朝向来模拟复位陀螺仪
    heading_offset = (static_cast<float>(imu_sensors.data.yaw) / 10.0) * ecl::pi / 180.0;  //????
  }

  void Xbot::getWheelJointStates(float &wheel_1_angle , float &wheel_1_angle_rate,
                                 float &wheel_2_angle , float &wheel_2_angle_rate,
                                 float &wheel_3_angle , float &wheel_3_angle_rate,
                                 float &wheel_4_angle , float &wheel_4_angle_rate)
  {//差分驱动里程计模型
    diff_drive.getWheelJointStates(wheel_1_angle , wheel_1_angle_rate,
                                   wheel_2_angle , wheel_2_angle_rate,
                                   wheel_3_angle , wheel_3_angle_rate,
                                   wheel_4_angle , wheel_4_angle_rate);
  }//将最后一次数据传入

/**
 * @brief Use the current sensor data (encoders and gyro 编码器和陀螺仪 ) to calculate an update for the odometry.更新里程计
 *
 * This fuses current sensor data with the last updated odometry state to produce the new
 * odometry state. This will be usually done in the slot callback to the stream_data signal.
 * 融合当前传感器数据和上一次里程计状态产生新的里程计状态
 *
 * It is important that this is called every time a data packet is received from the robot.
 * 每次从机器人接收到数据包时都会调用这一更新函数
 *
 * @param pose_update : return the pose updates in this variable.
 * @param pose_update_rates : return the pose update rates in this variable.
 */
  void Xbot::updateOdometry(ecl::Pose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates)
  {
    diff_drive.update(imu_sensors.data.timestamp, core_sensors.data.encoder1, core_sensors.data.encoder2,
                      core_sensors.data.encoder3, core_sensors.data.encoder4,
                      pose_update, pose_update_rates);
  }

/*****************************************************************************
 ** Commands
 *****************************************************************************/



  void Xbot::setBaseControl(const float &linear_velocity_x, const float &linear_velocity_y, const float &angular_velocity)
  {
    diff_drive.setVelocityCommands(linear_velocity_x, linear_velocity_y, angular_velocity);

  }


  void Xbot::setLiftControl(const unsigned char &height_percent)
  {
      sendCommand(Command::SetLiftHeightControl(height_percent)); //将高度数据转换成十六进制序列并写入串口
      HeightPercent = height_percent;

  }

  void Xbot::setPlatformCameraControl(const unsigned char &platform_degree, const unsigned char &camera_degree)
  {
      sendCommand(Command::SetPlatformAndCameraControl(platform_degree,camera_degree));
      PlatformDegree = platform_degree;
      CameraDegree = camera_degree;

  }

  void Xbot::resetXbot()
  {
      setBaseControl(0,0,0);
      setLiftControl(0);
      setPlatformCameraControl(90,90);
  }

  void Xbot::sendBaseControlCommand() //发送基本控制指令(线速度 角速度)
  {
    std::vector<float> velocity_commands_received; //收到的速度指令
    if( acceleration_limiter.isEnabled() ) {
      velocity_commands_received=acceleration_limiter.limit(diff_drive.pointVelocity()); //limit:两个command(线、角速度)组成的向量
    } else {                                                                             //pointVelocity:此刻速度
      velocity_commands_received=diff_drive.pointVelocity();
    }
    diff_drive.velocityCommands(velocity_commands_received);//将线角速度分别赋给cmd[0][1] 并将角速度转换为角度/s
    std::vector<float> velocity_commands = diff_drive.velocityCommands();
    std::cout << "linear_velocity_x: " << velocity_commands[0] << ", linear_velocity_y: " << velocity_commands[1]
              << ", angular_velocity: " << velocity_commands[2] << std::endl;
    if ((velocity_commands[0]!=0) || (velocity_commands[1]!=0) || (velocity_commands[2]!=0)){//有速度指令
//        std::ofstream fout;
//        fout.open("/tmp/2.txt");
//        fout << velocity_commands[0]; //<< "|" <<velocity_commands[1]<< "|" <<velocity_commands[2] <<std::endl;
//        fout << velocity_commands[1] <<std::endl;
//        fout << velocity_commands[2] <<std::endl;
//        fout.close();
        //return velocity_commands[0];
        sendCommand(Command::SetVelocityControl(velocity_commands[0], velocity_commands[1],
                                                velocity_commands[2])); //线速度 角速度 标识符分别赋值给相应分量

        //std::cout<<velocity_commands[0] << "|" <<velocity_commands[1]<< "|" <<velocity_commands[2] <<std::endl;
    }

/*
 *   //experimental; send raw control command and received command velocity
//  velocity_commands_debug=velocity_commands;
//  velocity_commands_debug.push_back((short)(velocity_commands_received[0]*1000.0));
//  velocity_commands_debug.push_back((short)(velocity_commands_received[1]*1000.0));
//  sig_raw_control_command.emit(velocity_commands_debug);*/
}

/**
 * @brief Send the prepared command to the serial port.
 * 向串口发送准备好的指令
 * Need to be a bit careful here, because we have no control over how the user
 * is calling this - they may be calling from different threads (this is so for
 * xbot_node), so we mutex(使用互斥锁) protect it here rather than relying on the user
 * to do so above.
 *
 * @param command : prepared command template (see Command's static member functions).
 */
  void Xbot::sendCommand(Command command)//传入的是command类中的data结构体数据类型
  {
    if( !is_alive || !is_connected ) {
      //need to do something
      sig_debug.emit("Device state is not ready yet.");
      if( !is_alive     ) sig_debug.emit(" - Device is not alive.");
      if( !is_connected ) sig_debug.emit(" - Device is not connected.");
      //std::cout << is_enabled << ", " << is_alive << ", " << is_connected << std::endl;
      return;
    }
    command_mutex.lock();
    xbot_command.resetBuffer(command_buffer); //初始化:帧头已经塞入，第三个单位(长度)为置0

    if (!command.serialise(command_buffer))//序列化  将数据转换成十六进制序列,从第四个单位开始逐个塞入
    {
      sig_error.emit("command serialise failed.");
    }
    command_buffer[2] = command_buffer.size() - 3; //序列长度
    unsigned char checksum = 0;
    for (unsigned int i = 2; i < command_buffer.size(); i++)
      checksum ^= (command_buffer[i]); //计算序列的最后一项(除了帧头外所有数据逐项异或)
    command_buffer.push_back(checksum);
  /*
   * check_device();
  //  for (int i = 0;i < command_buffer.size();i++)
  //    {
  //  //      std::cout <<std::hex<<command_buffer[i]<<std::endl;
  //        printf("%02x ",command_buffer[i]);

  //    }*/
      std::cout<<"-----------------------------------"<<std::endl;
    //将字符串写入串口,两个参数分别为字符串第一个字符和要写入的字符数量
    serial.write((const char*)&command_buffer[0], command_buffer.size());
    //发送数据
    sig_raw_data_command.emit(command_buffer);
    command_mutex.unlock();
  }

  bool Xbot::enable()
  {
    is_enabled = true;
    return true;
  }

  bool Xbot::disable()
  {
    setBaseControl(0.0f, 0.0f, 0.0f);
    sendBaseControlCommand();
    is_enabled = false;
    return true;
  }

/**
 * @brief Print a list of all relevant sigslot connections.
 *
 * This includes both the xbot driver signals as well as externally
 * connected slots. Useful for when you need to check if any of your
 * connections are dangling (often happens when you typo
 * the name of the sigslots connection).
 */

} // namespace xbot
