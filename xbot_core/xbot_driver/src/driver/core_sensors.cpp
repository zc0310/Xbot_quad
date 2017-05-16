/**
 * @file /xbot_driver/src/driver/core_sensors.cpp
 *
 * @brief Implementation of the core sensor packet data.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/packets/core_sensors.hpp"
#include "../../include/xbot_driver/packet_handler/payload_headers.hpp"
#include <time.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool CoreSensors::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
/*  buildBytes(Header::CoreSensors, byteStream);
//  buildBytes(length, byteStream);
//  buildBytes(data.time_stamp, byteStream);	//2
//  buildBytes(data.bumper, byteStream);		//1
//  buildBytes(data.wheel_drop, byteStream);	//1
//  buildBytes(data.cliff, byteStream);		//1
//  buildBytes(data.left_encoder, byteStream);	//2
//  buildBytes(data.right_encoder, byteStream);	//2
//  buildBytes(data.left_pwm, byteStream);	//1
//  buildBytes(data.right_pwm, byteStream);	//1
//  buildBytes(data.buttons, byteStream);		//1
//  buildBytes(data.charger, byteStream);		//1
//  buildBytes(data.battery, byteStream);		//1
//  buildBytes(data.over_current, byteStream);	//1 */

  return true;
}



bool CoreSensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream) //返回bool值
{//反序列化
/*
 *   std::cout<<byteStream.size()<<std::endl;

//  if (byteStream.size() < length+2)
//  {
//    std::cout<<"length:"<<(unsigned int)length<<std::endl;
//    std::cout << "bytestream.size:"<<byteStream.size()<<std::endl<<"xbot_node: xbot_default: deserialise failed. not enough byte stream." << std::endl;
//    return false;
//  }*/

  unsigned char header_id;
  buildVariable(header_id, byteStream);//将指令反解得出具体数据(驱动板发给上位机的的传感器数据) 此行意在得到标识符
  std::cout<<"header_id:"<<(unsigned int)header_id<<std::endl;
  if( header_id != Header::CoreSensors ) return false;

  unsigned char power_num;
  buildVariable(power_num , byteStream); //将byteStream数据依次赋给各变量然后推出
  build_special_variable(data.power_voltage , byteStream);
  std::cout<<"power_num:"<<(unsigned int) power_num<<std::endl;
  std::cout<<"power_voltage:"<<data.power_voltage<<std::endl;

  unsigned char infred_num;
  buildVariable(infred_num , byteStream); //没用红外传感器
  std::cout<<"infred_num:"<<(unsigned int) infred_num<<std::endl;
  /*buildVariable(data.infred_1,byteStream);
  buildVariable(data.infred_2,byteStream);
  std::cout<<"time:"<<time(0)<<"infred_1"<<data.infred_1<<std::endl;
  std::cout<<"time:"<<time(0)<<"infred_2"<<data.infred_2<<std::endl;*/


  unsigned char current_num;
  buildVariable(current_num,byteStream);
  build_special_variable(data.current_1 , byteStream);
  build_special_variable(data.current_2 , byteStream);
  build_special_variable(data.current_3 , byteStream);
  build_special_variable(data.current_4 , byteStream);
  build_special_variable(data.current_5 , byteStream);
  std::cout<<"current_num:"<<(unsigned int) current_num<<std::endl;
  std::cout<<"time:"<<time(0)<<" | current_1:"<<data.current_1<<std::endl;
  std::cout<<"time:"<<time(0)<<" | current_2:"<<data.current_2<<std::endl;
  std::cout<<"time:"<<time(0)<<" | current_3:"<<data.current_3<<std::endl;
  std::cout<<"time:"<<time(0)<<" | current_4:"<<data.current_4<<std::endl;
  std::cout<<"time:"<<time(0)<<" | current_5:"<<data.current_5<<std::endl;

  unsigned char echo_num;
  buildVariable(echo_num,byteStream);
  build_special_variable(data.echo_1 , byteStream);
  build_special_variable(data.echo_2 , byteStream);
  build_special_variable(data.echo_3 , byteStream);
  build_special_variable(data.echo_4 , byteStream);
  std::cout<<"echo_num:"<<(unsigned int) echo_num<<std::endl;
  std::cout<<"time:"<<time(0)<<" | echo_1:"<<data.echo_1<<std::endl;
  std::cout<<"time:"<<time(0)<<" | echo_2:"<<data.echo_2<<std::endl;
  std::cout<<"time:"<<time(0)<<" | echo_3:"<<data.echo_3<<std::endl;
  std::cout<<"time:"<<time(0)<<" | echo_4:"<<data.echo_4<<std::endl;

  unsigned char encoder_num;
  buildVariable(encoder_num, byteStream); //码盘路数 其后依次表示motor1/2/3码盘计数
  buildVariable(data.encoder1 , byteStream);
  buildVariable(data.encoder2 , byteStream);
  buildVariable(data.encoder3 , byteStream);
  buildVariable(data.encoder4 , byteStream);
  buildVariable(data.up_encoder, byteStream);
  std::cout<<"encoder_num:"<<(unsigned int) encoder_num<<std::endl;
  std::cout<<"time:"<<time(0)<<" | encoder1:"<<data.encoder1<<std::endl;
  std::cout<<"time:"<<time(0)<<" | encoder2:"<<data.encoder2<<std::endl;
  std::cout<<"time:"<<time(0)<<" | encoder3:"<<data.encoder3<<std::endl;
  std::cout<<"time:"<<time(0)<<" | encoder4:"<<data.encoder4<<std::endl;
  std::cout<<"time:"<<time(0)<<" | encoder_up:"<<data.up_encoder<<std::endl;


  /*std::cout<<"power:"<<data.power_voltage<<"|Echo1:"<<data.echo_1<<"|Echo2:"<<data.echo_2<<"|Echo3:"<<data.echo_3<<"|Echo4:"<<data.echo_4<<std::endl;*/
  return true;
}

void CoreSensors::build_special_variable(float &variable, ecl::PushAndPop<unsigned char> &byteStream)
{
    if (byteStream.size() < 2)
      return;

    unsigned char a, b;
    buildVariable(a,byteStream);
    buildVariable(b,byteStream);
    variable = ((unsigned int)(a&0x0f))/100.0; //计算点后第二位

    variable += ((unsigned int)(a>>4))/10.0;   //计算点后第一位

    variable += (unsigned int)(b&0x0f);        //计算个位

    variable += ((unsigned int)(b>>4))*10;     //计算十位


}


} // namespace xbot
