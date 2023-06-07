#include <chrono>
#include <memory>

#include <cstdio>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <jsoncpp/json/json.h>
#include <sstream>
#include <fstream>
#include "hf_interfaces/msg/stampstring.hpp"
//ultrasound library
#include <string>
#include <fcntl.h>
#include <termios.h> 
#include <errno.h>
#include <iostream>
#include <list>
#include <time.h>
#include <chrono>
#include <thread>
using namespace std;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node rosdriver("minimal_publisher");


  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ultrasound;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ultrasound_event;
  rclcpp::Publisher<hf_interfaces::msg::Stampstring>::SharedPtr publisher_car_speed;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_can;
  
  publisher_ultrasound = rosdriver.create_publisher<std_msgs::msg::String>("ultrasound_message", 10);
  publisher_ultrasound_event = rosdriver.create_publisher<std_msgs::msg::String>("ultrasound_event_message", 10);
  publisher_car_speed = rosdriver.create_publisher<hf_interfaces::msg::Stampstring>("car_speed_message", 10);
  publisher_can = rosdriver.create_publisher<std_msgs::msg::String>("car_can_message", 10);

  Json::Value root;
  float v_car;
  int s, nbytes0,nbytes1;
  struct sockaddr_can addr;
  struct ifreq ifr;

  struct can_frame frame[2] = {{0}};
  struct can_filter rfilter[1];
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
  strcpy(ifr.ifr_name,"can1");
  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与 can0 绑定

  //禁用过滤规则，本进程不接收报文，只负责发送
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
  //定义接收规则，只接收表示符等于 0x11 的报文
  rfilter[0].can_id = 0x11;
  rfilter[0].can_mask = CAN_SFF_MASK;
  //设置过滤规则
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
  //生成两个报文

  frame[0].can_id = 0x22;

  frame[0].can_dlc = 8;
  frame[0].data[0] =0x11;
  frame[0].data[1] =0x22;
  frame[0].data[2] =0x33;
  frame[0].data[3] =0x44;

  bool ttyUSB0_flag=true;
  bool ttyUSB1_flag=true;

  time_t t = time(0); 
  char date[32]={NULL};
  char time[32]={NULL};
  strftime(date, sizeof(date), "%Y-%m-%d",localtime(&t));
  strftime(time, sizeof(time), "%H:%M:%S",localtime(&t));
  std::string date_now=date;
  std::string time_now=time;
  string status_filepath="/home/nvidia/datalog/adas/system_log/"+date_now;
  string status_filename=status_filepath+"/"+"status.txt";
  string command_status;
  if (0 != access(status_filepath.c_str(), 0))
  {
    // 返回 0 表示创建成功，-1 表示失败
    command_status = "mkdir -p " + status_filepath;
    system(command_status.c_str());    	
  } 
  int fd0,fd1; /*File Descriptor*/
  Json::Value ultrasound;
  list<float> ultrasound_distance;
  std::string ultrasound_event_id;
  fd0 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  fd1 = open("/dev/ttyCH9344USB6", O_RDWR | O_NOCTTY | O_NDELAY);

  fcntl(fd0,F_SETFL,0);
  fcntl(fd1,F_SETFL,0);
  
  std::ofstream ofs;
  ofs.open(status_filename,std::ios::app);
  if (fd0 == -1) 
    {
      if (ttyUSB0_flag)
      {
        printf("\n  Error! in Opening ttyUSB0  ");
        ofs << "  Error! in Opening ttyUSB0.\n";
      }
      ttyUSB0_flag=false;
    }
  else
    {
      if (ttyUSB0_flag)
      {
        printf("\n  ttyUSB0 Opened Successfully ");
        ofs << "  ttyUSB0 Opened Successfully.\n";
      }
      ttyUSB0_flag=false;
    }
  if (fd1 == -1) 
    {
      if (ttyUSB1_flag)
      {
        printf("\n  Error! in Opening ttyCH9344USB6  ");
        ofs << "  Error! in Opening ttyCH9344USB6.\n";
      }
      ttyUSB1_flag=false;
    }
  else
    {
      if (ttyUSB1_flag)
      {
        printf("\n  ttyCH9344USB6 Opened Successfully ");
        ofs << "  ttyCH9344USB6 Opened Successfully.\n";
      }
      ttyUSB1_flag=false;
    }
  ofs.close();
  struct termios oldtio,SerialPortSettings; 

  tcgetattr(fd0, &oldtio); 
  tcgetattr(fd1, &oldtio); 

  //cfmakeraw(&SerialPortSettings);
  bzero(&SerialPortSettings,sizeof(SerialPortSettings));
  //tcflush(fd0,TCIOFLUSH);
  //tcflush(fd1,TCIOFLUSH);

  //设置波特率
  cfsetispeed(&SerialPortSettings, B9600); 
  cfsetospeed(&SerialPortSettings, B9600); 
  SerialPortSettings.c_cflag |= CREAD | CLOCAL; 
  SerialPortSettings.c_cc[VTIME]=10;
  SerialPortSettings.c_cc[VMIN]=24;
  tcflush(fd0,TCIOFLUSH);
  tcsetattr(fd0, TCSANOW, &SerialPortSettings);
  tcsetattr(fd1, TCSANOW, &SerialPortSettings);

  //定义传输内容
  char write_buffer0[8] = {0xAA,0xAA,0xAA,0xA5,0x01,0xAF};
  char write_buffer1[8] = {0xA5,0x56,0x01,0xFC};
  char buffer0[24];
  char buffer1[12];
  //传输字节数 
  int bytes_written0 = 0;
  int bytes_written1 = 0;

  int bytes_read0=0;
  int bytes_read1=0;
  //float v_car= msg->data; 
  while(rclcpp::ok())
  {
    nbytes0 = write(s, &frame[0],sizeof(frame[0]));//发送 frame[0]
    printf("Send Error frame[0]\n!");
    nbytes1 = read(s, &frame[1], sizeof(frame[1])); //接收报文

    
    float v_car=frame[1].data[0];
    auto string_car_speed_msg = hf_interfaces::msg::Stampstring();
    auto stamp = rclcpp::Clock().now();
    string_car_speed_msg.header.stamp.sec = stamp.seconds();
    string_car_speed_msg.header.stamp.nanosec = stamp.nanoseconds();
    string_car_speed_msg.data=v_car;
    publisher_car_speed->publish(string_car_speed_msg);
    auto string_can_msg = std_msgs::msg::String();

    //显示报文
    if(nbytes1 > 0)
    {
      printf("ID=0x%X DLC=%d data[0]=0x%X\n", frame[1].can_id,
          frame[1].can_dlc, frame[1].data[0]);
      root["car speed"]=Json::Value(frame[1].data[0]);
      root["car jiasudu"]=Json::Value(frame[1].data[1]);
      root["car dangwei"]=Json::Value(frame[1].data[2]);
      root["car zhidong"]=Json::Value(frame[1].data[3]);

      
      Json::FastWriter sw;
      string_can_msg.data=sw.write(root);    
    }
    else{
        string_can_msg.data="";
    }
    auto start = std::chrono::system_clock::now();
    //串口写数据
    bytes_written0 = write(fd0, write_buffer0, sizeof(write_buffer0));
    bytes_written1 = write(fd1, write_buffer1, sizeof(write_buffer1));  
    //tcdrain(fd0);
    //tcdrain(fd1);                                                                                                   
    printf("\n  %s written to ttyUSB0", write_buffer0);
    printf("\n  %d Bytes written to ttyUSB0", bytes_written0);
    printf("\n +----------------------------------+\n\n");
    sleep(0.3);
    auto string_ultrasound_msg = std_msgs::msg::String();  
    if(bytes_written0!=0 && bytes_written1!=0)
    {
      //读出数据
      bytes_read0=read(fd0,buffer0,sizeof(buffer0));
      bytes_read1=read(fd1,buffer1,sizeof(buffer1));
      auto end = std::chrono::system_clock::now();
      std::cerr <<"串口 seconds"<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
      std::cerr <<bytes_read0<< std::endl;
      std::cerr <<bytes_read1<< std::endl;
      float front_distance1=buffer0[4]*0.1;
      float front_distance2=buffer0[10]*0.1;
      float front_distance3=buffer0[16]*0.1;
      float front_distance4=buffer0[22]*0.1;
      //float back_distance1=buffer1[4];
      //float back_distance2=buffer1[10]*0.1;
      //float back_distance3=buffer1[16]*0.1;
      //float back_distance4=buffer1[22]*0.1;

      ultrasound_distance.emplace_back(front_distance1);
      ultrasound_distance.emplace_back(front_distance2);
      ultrasound_distance.emplace_back(front_distance3);
      ultrasound_distance.emplace_back(front_distance4);

      //ultrasound_distance.emplace_back(back_distance1);
      //ultrasound_distance.emplace_back(back_distance2);
      //ultrasound_distance.emplace_back(back_distance3);
      //ultrasound_distance.emplace_back(back_distance4);

      printf("ultrasound 1 distance: %.2f\n ", front_distance1);
      printf("ultrasound 2 distance: %.2f\n ", front_distance2);
      printf("ultrasound 3 distance: %.2f\n ", front_distance3);
      printf("ultrasound 4 distance: %.2f\n ", front_distance4);
      //printf("ultrasound 5 distance: %.2f\n ", distance5);
      ultrasound["front 1"]=Json::Value(front_distance1);
      ultrasound["front 2"]=Json::Value(front_distance2);
      ultrasound["front 3"]=Json::Value(front_distance3);
      ultrasound["front 4"]=Json::Value(front_distance4);

      //ultrasound["back 1"]=Json::Value(back_distance1);
      //ultrasound["back 2"]=Json::Value(back_distance2);
      //ultrasound["back 3"]=Json::Value(back_distance3);
      //ultrasound["back 4"]=Json::Value(back_distance4);          
      Json::FastWriter uw;
      string_ultrasound_msg.data=uw.write(ultrasound);
      ultrasound_distance.sort();
      float min_distance=ultrasound_distance.front();
      printf("min distance: %.2f\n ", min_distance);
      if (min_distance > 1.8 && min_distance <= 2.3)
      { ultrasound_event_id="10000001";
        std::cerr<<"超声波雷达提示"<<std::endl;}
      else if (min_distance > 1.3 && min_distance <= 1.8)
      { ultrasound_event_id="10000002";
        std::cerr<<"超声波雷达预警"<<std::endl;}
      else if (min_distance > 0.8 && min_distance <= 1.3)
      {ultrasound_event_id="10000003";
        std::cerr<<"超声波雷达报警"<<std::endl;}
      else if (min_distance >= 0.3 && min_distance <= 0.8)
      { ultrasound_event_id="10000004";
        std::cerr<<"超声波雷达制动"<<std::endl;}
      if (v_car>10)
      {
        auto string_ultrasound_event_msg = std_msgs::msg::String();
        string_ultrasound_event_msg.data=ultrasound_event_id;
        publisher_ultrasound_event->publish(string_ultrasound_event_msg);
      }
      for(int i=0; i<bytes_read0; i++)
      {
          //16进制的方式打印到屏幕
          //string hex_16=std::to_string(buffer[i] & 0xff);
          int int_10=buffer0[i];
          //std::cerr << std::hex << (buffer[i] & 0xff) << " ";
          printf("%02X ", buffer0[i]);
          //printf("%d ", int_10);
      }
      for(int i=0; i<bytes_read1; i++)
      {
          //16进制的方式打印到屏幕
          //string hex_16=std::to_string(buffer[i] & 0xff);
          int int_10=buffer1[i];
          //std::cerr << std::hex << (buffer[i] & 0xff) << " ";
          printf("%02X ", buffer1[i]);
          printf("%d ", int_10);
      }
      std::cerr << std::endl;
      memset(buffer0,0,sizeof(buffer0));
    }
    ultrasound_distance.clear();
    publisher_ultrasound->publish(string_ultrasound_msg);
    publisher_can->publish(string_can_msg);
    //rclcpp::spin();
    }

  
  rclcpp::shutdown();
}

