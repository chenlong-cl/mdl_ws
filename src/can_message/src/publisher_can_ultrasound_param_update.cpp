#include <chrono>
#include <memory>
#include <bitset>
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
#include "custom_interfaces/msg/stampstring.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
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
//using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

int can_adas_ultrasound;
class CanPublisher : public rclcpp::Node
{
public:
  CanPublisher()
  : Node("can_publisher"), count_(0)
  {  
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
    strcpy(ifr.ifr_name,"can1");
    ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备
    
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与 can0 绑定 
      
    //zhw
    dsm_sub.subscribe(this, "/dsm/can_message");
    adas_sub.subscribe(this, "/adas/can_message");
    sync.reset(new Sync(MySyncPolicy(10), dsm_sub, adas_sub));
    sync->registerCallback(std::bind(&CanPublisher::can_pub_callback, this, std::placeholders::_1, std::placeholders::_2)); 
    publisher_can = this->create_publisher<std_msgs::msg::String>("car_can_message", 10);
    publisher_param = this->create_publisher<std_msgs::msg::String>("param_update_message", 10);
    //publisher_car_speed = this->create_publisher<custom_interfaces::msg::Stampstring>("car_speed_message", 10);
    publisher_can_to_system = this->create_publisher<custom_interfaces::msg::Stampstring>("can_system_message", 10);  
    
    can_ = this->create_wall_timer(
      20ms, std::bind(&CanPublisher::can_callback, this)); 
    can_param_update = this->create_wall_timer(
      20s, std::bind(&CanPublisher::param_update_callback, this)); 

  }

private:
  int s, nbytes0,nbytes1,nbytes2,nbytes3;
  struct sockaddr_can addr;
  struct ifreq ifr;
  //struct can_frame frame_send[2]={{0}};

  //zhw
  std::string dsm_can_message, adas_can_message;
  std::string msg1_dsm;
  std::string msg2_adas;
  Json::Reader dsm_can_reader;
  Json::Reader adas_can_reader;
  Json::Value can_dsm;
  Json::Value can_adas;

  typedef message_filters::sync_policies::ApproximateTime<custom_interfaces::msg::Stampstring, custom_interfaces::msg::Stampstring> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync;
  message_filters::Subscriber<custom_interfaces::msg::Stampstring> dsm_sub;
  message_filters::Subscriber<custom_interfaces::msg::Stampstring> adas_sub;

  rclcpp::TimerBase::SharedPtr can_;
  rclcpp::TimerBase::SharedPtr can_param_update;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_can;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_param;
  rclcpp::Publisher<custom_interfaces::msg::Stampstring>::SharedPtr publisher_can_to_system;

  size_t count_;
  
  void param_update_callback()
  {
    Json::Value param;
    Json::Value adas,dsm;
    auto string_can_msg = std_msgs::msg::String();
    adas["nms_yolo_threshold"]=Json::Value(0.5);
    adas["conf"]=Json::Value(0.60);
    adas["nms_pp_threshold"]=Json::Value(0.01);
    adas["score_threshold"]=Json::Value(0.45);
    
    dsm["dsm_conf"]=Json::Value(0.5);
    dsm["dsm_nms"]=Json::Value(0.45);
    dsm["driver_ratio_"]=Json::Value(0.4);
    dsm["head_ratio_"]=Json::Value(0.3);
    
    param["adas"]=adas;
    param["dsm"]=dsm;
    
    Json::FastWriter sw;
    string_can_msg.data=sw.write(param); 
    publisher_param->publish(string_can_msg);
  }
  void can_callback()
  {
    Json::Value root;
    Json::Value can_to_system;
    struct can_frame frame[4] = {{0}};
    struct can_filter rfilter[3];
    //禁用过滤规则，本进程不接收报文，只负责发送
    //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //定义接收规则，只接收表示符等于 can_id 的报文
    rfilter[0].can_id = 0x180028D0;
    rfilter[1].can_id = 0x180D20E0;
    rfilter[2].can_id = 0x180128D0;

    //std::cerr<<rfilter[0].can_id<<std::endl;
    rfilter[0].can_mask = CAN_EFF_MASK;
    rfilter[1].can_mask = CAN_EFF_MASK;
    rfilter[2].can_mask = CAN_EFF_MASK;

    //设置过滤规则
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    //生成两个报文

    /*frame[0].can_id = 0x22;

    frame[0].can_dlc = 8;
    frame[0].data[0] =0x11;
    frame[0].data[1] =0x22;
    frame[0].data[2] =0x33;
    frame[0].data[3] =0x44;*/

    //nbytes0 = write(s, &frame[0],sizeof(frame[0]));//发送 frame[0]
    //printf("Send Error frame[0]\n!");
    
    nbytes1 = read(s, &frame[1], sizeof(frame[1])); //接收报文
    //std::cerr<<(frame[1].can_id & CAN_EFF_MASK)<<std::endl;
    printf("<0x%08X> ", frame[1].can_id & CAN_EFF_MASK);
    std::bitset<8> xingchezhuangtai;
    int v_car_decimal;
    int gear_d,gear_r,gear_n,qianyin,brake,gear_p;
    if (("0x%08X",(frame[1].can_id & CAN_EFF_MASK))==0x180028D0){
      printf("===1====\n");
      xingchezhuangtai=bitset<8>(frame[1].data[0]); // hex to bit
      v_car_decimal=(frame[1].data[3] << 8) | frame[1].data[4]; //frame[1].data[3]=0x01,frame[1].data[4]=0x2C,v_car_dec=0x012C(hex) -> 300(dec)
      gear_d=xingchezhuangtai[2];
      gear_r=xingchezhuangtai[3];
      gear_n=xingchezhuangtai[4];
      qianyin=xingchezhuangtai[5];
      brake=xingchezhuangtai[6];
      gear_p=xingchezhuangtai[7];
      std::cerr<<"D0 in bit: "<<xingchezhuangtai<<std::endl;
      std::cerr<<"v_car_decimal: "<<v_car_decimal<<std::endl;
      printf("data=0x%X\n", frame[1].data[0]);
      printf("data=0x%X\n", frame[1].data[1]);
      printf("data=0x%X\n", frame[1].data[2]);
      printf("data=0x%X\n", frame[1].data[3]); //car speed high level
      printf("data=0x%X\n", frame[1].data[4]); //car speed low level
      printf("data=0x%X\n", frame[1].data[5]);
      printf("data=0x%X\n", frame[1].data[6]);}
    nbytes2 = read(s, &frame[2], sizeof(frame[2])); //接收报文
    int belt,chair;
    if (("0x%08X",(frame[2].can_id & CAN_EFF_MASK))==0x180D20E0){
      printf("===2====\n");
      belt=frame[2].data[1];
      chair=frame[2].data[2];
      printf("data=0x%X\n", frame[2].data[0]);
      printf("data=0x%X\n", frame[2].data[1]);
      printf("data=0x%X\n", frame[2].data[2]);
      printf("data=0x%X\n", frame[2].data[3]);}
    nbytes3 = read(s, &frame[3], sizeof(frame[3])); //接收报文 
    if (("0x%08X",(frame[3].can_id & CAN_EFF_MASK))==0x180128D0){
      printf("===3====\n");
      printf("data=0x%X\n", frame[3].data[0]);
      printf("data=0x%X\n", frame[3].data[1]);
      printf("data=0x%X\n", frame[3].data[2]);
      printf("data=0x%X\n", frame[3].data[3]);}
    
    auto string_can_msg = std_msgs::msg::String();
    float v_car=v_car_decimal*0.1*1000/3600; // km/h -> m/s
    can_to_system["speed"]=Json::Value(v_car);
    can_to_system["gear_d"]=Json::Value(gear_d);
    can_to_system["gear_r"]=Json::Value(gear_r);
    can_to_system["gear_n"]=Json::Value(gear_n);
    can_to_system["gear_p"]=Json::Value(gear_p);
    can_to_system["qianyin"]=Json::Value(qianyin);
    can_to_system["brake"]=Json::Value(brake);
    can_to_system["belt"]=Json::Value(belt);
    can_to_system["chair"]=Json::Value(chair);

    auto can_to_system_msg = custom_interfaces::msg::Stampstring();
    // auto stamp = rclcpp::Clock().now();
    // can_to_system_msg.header.stamp.sec = stamp.seconds();
    // can_to_system_msg.header.stamp.nanosec = stamp.nanoseconds();
    auto stamp = this->get_clock()->now();
    can_to_system_msg.header.stamp = stamp;
    Json::FastWriter cw;
    can_to_system_msg.speed=v_car;
    can_to_system_msg.data=cw.write(can_to_system);
    publisher_can_to_system->publish(can_to_system_msg);

    root["dsm"]=Json::Value("dsm");
    //显示报文
    if(nbytes1 > 0)
    {
      //printf("ID=0x%X DLC=%d data[0]=0x%X\n", frame[1].can_id,
      //    frame[1].can_dlc, frame[1].data[0]);
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
    publisher_can->publish(string_can_msg);
  }
  
  void can_pub_callback(const custom_interfaces::msg::Stampstring::ConstSharedPtr dsm_msg, const custom_interfaces::msg::Stampstring::ConstSharedPtr adas_msg)
  {
    try{
    auto start_time = std::chrono::high_resolution_clock::now();
    
// 在这里执行需要测量时间的语句

    struct can_frame frame_send[2]={{0}};
    //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    int idx_dsm = -1; 
    int idx_adas = -1;
    int brake_ratio;
    int can_dsm_str0,can_dsm_str1,can_dsm_str2,can_dsm_str3,can_dsm_str4,can_dsm_str5,can_dsm_str6;
    int can_adas_brake,can_adas_warning,can_adas_shineng,can_adas_chaoyue,can_adas_error;
    dsm_can_message = dsm_msg->data;
    if (dsm_can_reader.parse(dsm_can_message, can_dsm))
    {
       RCLCPP_INFO(this->get_logger(), "Received messages: %s", msg1_dsm.c_str());
       can_dsm_str0 = can_dsm["0"].asInt();
       can_dsm_str1 = can_dsm["1"].asInt();
       can_dsm_str2 = can_dsm["2"].asInt();
       can_dsm_str3 = can_dsm["3"].asInt();
       can_dsm_str4 = can_dsm["4"].asInt();
       can_dsm_str5 = can_dsm["5"].asInt();
       can_dsm_str6 = can_dsm["6"].asInt();
       // std::cerr<< "dsm_can_message: " << can_dsm_str0 << can_dsm_str1 << can_dsm_str2 << can_dsm_str3 <<std::endl;
    std::cerr<< "dsm_brake_pedal:  " <<can_dsm["0"] << std::endl;
    std::cerr<< "dsm_brake_instruction:  " <<can_dsm["1"] << std::endl;
    std::cerr<< "dsm_park_instruction:  " <<can_dsm["2"]<< std::endl;
    std::cerr<< "dsm_adas:  " <<can_dsm["3"]<< std::endl;
    std::cerr<< "dsm_danger_action:  " <<can_dsm["4"] << std::endl;
    std::cerr<< "dsm_driver_leave:  " <<can_dsm["5"]<< std::endl;
    std::cerr<< "dsm_radar:  " <<can_dsm["6"] << std::endl;

    }
    adas_can_message = adas_msg->data;
      if (adas_can_reader.parse(adas_can_message, can_adas))
    {
       RCLCPP_INFO(this->get_logger(), "Received messages: %s", msg2_adas.c_str());
       can_adas_brake = can_adas["0"].asInt();
       can_adas_shineng = can_adas["1"].asInt();
       can_adas_chaoyue = can_adas["2"].asInt();
       can_adas_warning = can_adas["3"].asInt();
       can_adas_error = can_adas["4"].asInt();
       //can_adas_ultrasound = can_adas["5"].asInt();
    std::cerr<< "brake_pedal:  " <<can_adas["0"]<< std::endl;
    std::cerr<< "shineng_kaiguan:  " <<can_adas["1"]<< std::endl;
    std::cerr<< "chaoyue_kaiguan:  " <<can_adas["2"] << std::endl;
    std::cerr<< "adas_warning:  " <<can_adas["3"]<< std::endl;
    std::cerr<< "camera_status:  " <<can_adas["4"]<< std::endl;
    std::cerr<< "laser_status:  " <<can_adas["5"] << std::endl;
    } 
    
    if (can_dsm_str0>can_adas_brake){
    	brake_ratio=can_dsm_str0;
    }
    else
    	{brake_ratio=can_adas_brake;}
    	
    frame_send[0].can_id = 0x180B18B0;

    frame_send[0].can_dlc = 8;
    frame_send[0].data[0] =brake_ratio;
    frame_send[0].data[1] =can_dsm_str1;
    frame_send[0].data[2] =can_dsm_str2;
    frame_send[0].data[3] =can_adas_warning;
    frame_send[0].data[4] =can_dsm_str4;
    frame_send[0].data[5] =can_dsm_str5;
    frame_send[0].data[6] =can_adas_ultrasound;
    frame_send[0].data[7] =can_dsm_str6;
    
    frame_send[1].can_id = 0x180B18B1;
    frame_send[1].can_dlc = 3;

    frame_send[1].data[0] =can_adas_shineng;
    frame_send[1].data[1] =can_adas_chaoyue;
    frame_send[1].data[2] =can_adas_error;  
    
    if(write(s,&frame_send[0],sizeof(can_frame)) != sizeof(struct can_frame)){
      RCLCPP_INFO(this->get_logger(),"write error!");}
    if(write(s,&frame_send[1],sizeof(can_frame)) != sizeof(struct can_frame)){
      RCLCPP_INFO(this->get_logger(),"write error!");}
    //nbytes0 = write(s, &frame_send[0],sizeof(frame_send[0]));//发送 frame[0]
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    std::cerr << "Time taken by statement: " << duration.count() << " microseconds" << std::endl;
    }
    catch (std::exception e){
    std::cerr<<"execption has been catched"<<std::endl;
    return; }
  }

};

class UltrasoundPublisher : public rclcpp::Node
{
public:
  UltrasoundPublisher()
  : Node("ultrasound_publisher"), count_(0)
  {
    publisher_ultrasound = this->create_publisher<std_msgs::msg::String>("ultrasound_message", 10);
    publisher_ultrasound_event = this->create_publisher<std_msgs::msg::String>("ultrasound_event_message", 10);
    
    fd0 = open("/dev/ttyCH9344USB1", O_RDWR | O_NOCTTY | O_NDELAY);
    fd1 = open("/dev/ttyCH9344USB5", O_RDWR | O_NOCTTY | O_NDELAY);

    fcntl(fd0,F_SETFL,0);
    fcntl(fd1,F_SETFL,0);
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
    ultrasound_ = this->create_subscription<custom_interfaces::msg::Stampstring>(
      "can_system_message", 10, std::bind(&UltrasoundPublisher::ultrasound_callback, this, std::placeholders::_1));
  }
private:
  bool ttyUSB0_flag=true;
  bool ttyUSB1_flag=true;
  int fd0,fd1; /*File Descriptor*/
  std::shared_ptr<spdlog::logger> logger;
  int count_usb1=0;
  int count_usb2=0;
  void ultrasound_callback(const custom_interfaces::msg::Stampstring::ConstSharedPtr msg)
  {
    Json::Value ultrasound;
    list<float> ultrasound_distance;
    std::string ultrasound_event_id;

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

    //const std::string log_filepathname = log_filedir_prefix + log_time_tmp + "/camera_ros.log";
    //logger = spdlog::basic_logger_mt("basic_logger", status_filename);
    std::ofstream ofs;
    ofs.open(status_filename,std::ios::app);
    if (fd0 == -1) 
      {
       if (count_usb1%100==0){
          ofs << time_now << ":";
          ofs << "  Error! in Opening ttyCH9344USB1.\n";
	}
	count_usb1+=1;

      }
    else
      {
        if (ttyUSB0_flag)
        {
          printf("\n  ttyCH9344USB1 Opened Successfully ");
          ofs << "  ttyCH9344USB1 Opened Successfully.\n";
        }
        ttyUSB0_flag=false;
        
      }
    if (fd1 == -1) 
      {
	if (count_usb2%100==0){
          ofs << time_now << ":";
          ofs << "  Error! in Opening ttyCH9344USB5.\n";}
	count_usb2+=1;
      }
    else
      {
        if (ttyUSB1_flag)
        {
          printf("\n  ttyCH9344USB5 Opened Successfully ");
          ofs << "  ttyCH9344USB5 Opened Successfully.\n";
        }
        ttyUSB1_flag=false;
        
      }
    ofs.close();

    //定义传输内容
    char write_buffer0[8] = {0xAA,0xAA,0xAA,0xA5,0x01,0xAF};
    char write_buffer1[8] = {0xAA,0xAA,0xAA,0xA5,0x01,0xAF};
    char buffer0[24];
    char buffer1[24];
    //传输字节数 
    int bytes_written0 = 0;
    int bytes_written1 = 0;

    int bytes_read0=0;
    int bytes_read1=0;
    //size_t n = fd.available();
    float v_car=msg->speed;
    //printf("%f ", duration);    
 
      auto start = std::chrono::system_clock::now();
      //串口写数据
      bytes_written0 = write(fd0, write_buffer0, sizeof(write_buffer0));
      bytes_written1 = write(fd1, write_buffer1, sizeof(write_buffer1));  
      //tcdrain(fd0);
      //tcdrain(fd1);                                                                                                   
      printf("\n  %s written to ttyUSB0", write_buffer0);
      printf("\n  %d Bytes written to ttyUSB0", bytes_written0);
      printf("\n +----------------------------------+\n\n");
      sleep(0.5);
      auto string_ultrasound_msg = std_msgs::msg::String();  
      if(bytes_written0!=0 && bytes_written1!=0)
        {
          //读出数据
          bytes_read0=read(fd0,buffer0,sizeof(buffer0));
          bytes_read1=read(fd1,buffer1,sizeof(buffer1));
          auto end = std::chrono::system_clock::now();
          // std::cerr <<"串口 seconds"<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
          // std::cerr <<bytes_read0<< std::endl;
          // std::cerr <<bytes_read1<< std::endl;
          float front_distance1=buffer0[4]*0.1;
          float front_distance2=buffer0[10]*0.1;
          float front_distance3=buffer0[16]*0.1;
          float front_distance4=buffer0[22]*0.1;
          
          float back_distance1=buffer1[4]*0.1;
          float back_distance2=buffer1[10]*0.1;
          float back_distance3=buffer1[16]*0.1;
          float back_distance4=buffer1[22]*0.1;

          ultrasound_distance.emplace_back(front_distance1);
          ultrasound_distance.emplace_back(front_distance2);
          ultrasound_distance.emplace_back(front_distance3);
          ultrasound_distance.emplace_back(front_distance4);

          ultrasound_distance.emplace_back(back_distance1);
          ultrasound_distance.emplace_back(back_distance2);
          ultrasound_distance.emplace_back(back_distance3);
          ultrasound_distance.emplace_back(back_distance4);

          //printf("ultrasound 1 distance: %.2f\n ", front_distance1);
          //printf("ultrasound 2 distance: %.2f\n ", front_distance2);
          //printf("ultrasound 3 distance: %.2f\n ", front_distance3);
          //printf("ultrasound 4 distance: %.2f\n ", front_distance4);
          //printf("ultrasound 5 distance: %.2f\n ", distance5);
          ultrasound["front 1"]=Json::Value(front_distance1);
          ultrasound["front 2"]=Json::Value(front_distance2);
          ultrasound["front 3"]=Json::Value(front_distance3);
          ultrasound["front 4"]=Json::Value(front_distance4);

          ultrasound["back 1"]=Json::Value(back_distance1);
          ultrasound["back 2"]=Json::Value(back_distance2);
          ultrasound["back 3"]=Json::Value(back_distance3);
          ultrasound["back 4"]=Json::Value(back_distance4);          
          Json::FastWriter uw;
          string_ultrasound_msg.data=uw.write(ultrasound);
          ultrasound_distance.sort();
          float min_distance=ultrasound_distance.front();
          // printf("min distance: %.2f\n ", min_distance);
          if (v_car<=2.8)
          {
            if (min_distance > 1.8 && min_distance <= 2.3)
            { ultrasound_event_id="10000001";
              can_adas_ultrasound=1;
              std::cerr<<"超声波雷达提示"<<std::endl;}
            else if (min_distance > 1.3 && min_distance <= 1.8)
            { ultrasound_event_id="10000002";
              can_adas_ultrasound=2;
              std::cerr<<"超声波雷达预警"<<std::endl;}
            else if (min_distance > 0.8 && min_distance <= 1.3)
            { ultrasound_event_id="10000003";
              can_adas_ultrasound=3;
              std::cerr<<"超声波雷达报警"<<std::endl;}
            else if (min_distance >= 0.3 && min_distance <= 0.8)
            { ultrasound_event_id="10000004";
              can_adas_ultrasound=4;
              std::cerr<<"超声波雷达制动"<<std::endl;}
            else 
   	    { can_adas_ultrasound=0;
      	      std::cerr<<"超声波雷达未检测到障碍物"<<std::endl;}
            auto string_ultrasound_event_msg = std_msgs::msg::String();
            string_ultrasound_event_msg.data=ultrasound_event_id;
            publisher_ultrasound_event->publish(string_ultrasound_event_msg);
          }
          else
            {can_adas_ultrasound=0;}
          for(int i=0; i<bytes_read0; i++)
          {
              //16进制的方式打印到屏幕
              //string hex_16=std::to_string(buffer[i] & 0xff);
              //std::cerr << "front ultrasound value " << std::endl;
              int int_10=buffer0[i];
              //std::cerr << std::hex << (buffer[i] & 0xff) << " ";
              printf("%02X ", buffer0[i]);
              //printf("%d ", int_10);
          }
          for(int i=0; i<bytes_read1; i++)
          {
              //16进制的方式打印到屏幕
              //std::cerr << "back ultrasound value " << std::endl;
              //string hex_16=std::to_string(buffer[i] & 0xff);
              int int_10=buffer1[i];
              //std::cerr << std::hex << (buffer[i] & 0xff) << " ";
              printf("%02X ", buffer1[i]);
              //printf("%d ", int_10);
          }
            // std::cerr << std::endl;
            memset(buffer0,0,sizeof(buffer0));
        }
      ultrasound_distance.clear();
      ultrasound.clear();
      publisher_ultrasound->publish(string_ultrasound_msg);

    
    //close(s);
    //close(fd0); 
    //close(fd1); 
  }
  //rclcpp::TimerBase::SharedPtr ultrasound_;
  rclcpp::Subscription<custom_interfaces::msg::Stampstring>::ConstSharedPtr ultrasound_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ultrasound;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ultrasound_event;

  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto can_node=std::make_shared<CanPublisher>();
  auto ultrasound_node=std::make_shared<UltrasoundPublisher>();
  rclcpp::executors::MultiThreadedExecutor exector;
  exector.add_node(can_node);
  exector.add_node(ultrasound_node);
  exector.spin();
  //rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

