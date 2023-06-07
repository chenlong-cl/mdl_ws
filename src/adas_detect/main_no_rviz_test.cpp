/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define BOOST_BIND_NO_PLACEHOLDERS
#define YOLOV5_TENSORRT_H
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ctime>
#include <time.h>
#include "cuda_runtime.h"
#include <chrono>
#include <memory>
#include <vector>
#include <iomanip>
#include <map>
#include <algorithm>
#include <cassert>
#include <string>
#include <math.h>
#include "./params.h"
#include "pointpillar.h"  
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "custom_interfaces/msg/stampstring.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
//ultrasound library

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pcl_conversions/pcl_conversions.h"
#include <unistd.h>
#include <sys/stat.h>
//yolov5
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "yolov5.h"
#include "yoloparam.h"

//message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace Eigen;

typedef struct
{
    string tmp;
    string time_;
    string event_id;
    Mat out_img;
} save_message;

template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

class PointPillarsROS : public rclcpp::Node
{
public: 
  yoloparam YP;
  void* YParam;

  PointPillarsROS()
  : Node("pointpillars_ros")
  {

  //algorithm pointpillar related params

  this->declare_parameter<std::string>("data_type", "fp16");
  this->declare_parameter<string>("point_sub_topic_name", "/c16/lslidar_point_cloud");
  this->declare_parameter<float>("intensity_scale", 1.0);
  this->declare_parameter<float>("score_threshold", 0.4);
  this->declare_parameter<float>("nms_pp_threshold", 0.01);
  this->declare_parameter<std::string>("pointpillar_onnx_file", "../model/pointpillar.onnx");
  // filter point range
  this->declare_parameter<float>("x_min", -10.0);
  this->declare_parameter<float>("x_max", 10.0);
  this->declare_parameter<float>("y_min", -30);
  this->declare_parameter<float>("y_max", -0.5);
  this->declare_parameter<float>("z_min", -0.8);
  this->declare_parameter<float>("z_max", 5.0);

//apa_speed


  //algorithm yolov5 related params
  this->declare_parameter<string>("image_sub_topic_name", "/adas/video_frames");
  this->declare_parameter<int>("device", 0);
  this->declare_parameter<double>("nms_yolo_threshold", 0.45);
  this->declare_parameter<double>("conf", 0.5);
  this->declare_parameter<int>("batch_size", 1);
  this->declare_parameter<int>("input_h", 640);
  this->declare_parameter<int>("input_w", 640);
  this->declare_parameter<int>("class_num", 68);
  this->declare_parameter<string>("engine_dir", "../engine/best.engine");

  
  //pointpillar
  data_type = this->get_parameter("data_type").as_string();
  std::string point_sub_topic_name=this->get_parameter("point_sub_topic_name").as_string();
  intensity_scale = this->get_parameter("intensity_scale").as_double();
  score_threshold=this->get_parameter("score_threshold").as_double();
  nms_threshold=this->get_parameter("nms_pp_threshold").as_double();
  pp_onnx_file_=this->get_parameter("pointpillar_onnx_file").as_string();
  RCLCPP_INFO(this->get_logger(),"init start %s", pp_onnx_file_.c_str());

  x_min=this->get_parameter("x_min").as_double();
  x_max=this->get_parameter("x_max").as_double();
  y_min=this->get_parameter("y_min").as_double();
  y_max=this->get_parameter("y_max").as_double();
  z_min=this->get_parameter("z_min").as_double();
  z_max=this->get_parameter("z_max").as_double();
  //yolov5
  std::string image_sub_topic_name=this->get_parameter("image_sub_topic_name").as_string();
  this->get_parameter("device", YP.DEVICE);
  this->get_parameter("nms_yolo_threshold", YP.NMS_THRESH);
  this->get_parameter("conf", YP.CONF_THRESH);
  this->get_parameter("batch_size", YP.BATCH_SIZE);
  this->get_parameter("input_h", YP.INPUT_H);
  this->get_parameter("input_w", YP.INPUT_W);
  this->get_parameter("class_num", YP.CLASS_NUM);
  this->get_parameter("engine_dir", YP.ENGINE_DIR);

  //new add for topic_callback(iamge,point,speed)
  this->declare_parameter<string>("speed_sub_topic_name", "/can_system_message");
  std::string speed_sub_topic_name=this->get_parameter("speed_sub_topic_name").as_string();

  RCLCPP_INFO(this->get_logger(), "class_num: '%d'", YP.CLASS_NUM);
  YParam = &YP;
  
  cudaStream_t stream = NULL;
  pointpillar= new PointPillar(pp_onnx_file_, stream);

  yolov5 = new Yolov5(YParam);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.deadline(std::chrono::milliseconds(20));
  
  time_t t = time(0); 
  char date[32]={NULL};
  char time[32]={NULL};
  strftime(date, sizeof(date), "%Y-%m-%d",localtime(&t));
  strftime(time, sizeof(time), "%H:%M:%S",localtime(&t));
  std::string date_=date;
  std::string time_=time; 
  int coder=VideoWriter::fourcc('X','V','I','D');//选择编码格式
  double fps=10.0;//设置视频帧率

  string video_filepath="/home/nvidia/datalog/adas/video/"+date_;
  string command_video;
  if (0 != access(video_filepath.c_str(), 0))
  {
	// 返回 0 表示创建成功，-1 表示失败
	command_video = "mkdir -p " + video_filepath;
	system(command_video.c_str());    	
  }

  string filename=video_filepath+"/"+time_+".avi";//保存的视频文件名称
  writer.open(filename,coder,fps,Size(640,480),true);//创建保存视频文件的视频流

  
  lidar_to_camera_matrix=load_csv<Matrix4d>("/home/nvidia/mdl_ws/src/etc/yaml/extrinsic.csv");
  imagesubfilter.subscribe(this,image_sub_topic_name);
  pointsubfilter.subscribe(this,point_sub_topic_name);
  speedsubfilter.subscribe(this,speed_sub_topic_name);

  sync.reset(new Sync(MySyncPolicy(10),imagesubfilter,pointsubfilter,speedsubfilter));
  sync->registerCallback(std::bind(&PointPillarsROS::topic_callback, 
               this, _1, _2,_3));

  fusion_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
      "/adas/fusion_obj_detection", 3);
  compressed_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("/adas/fusion_compressed_obj_detection", qos);
  string_event_pub= this->create_publisher<std_msgs::msg::String>("/event_messages", 700);
  string_warn_pub= this->create_publisher<std_msgs::msg::String>("/warn_messages", 700);
  string_speed_pub= this->create_publisher<std_msgs::msg::String>("/speed_messages", 700);
  string_can_pub= this->create_publisher<custom_interfaces::msg::Stampstring>("/adas/can_message", 700);

  }
  


private:

  bool pic_flag=true;
  bool point_flag=true;
  bool model_flag=true;
  bool ttyUSB0_flag=true;
  bool ttyUSB1_flag=true;

  int fd0,fd1;
  std::string data_type;
  std::string date_;
  std::string time_;
  float nms_threshold;
  float score_threshold;
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float z_min;
  float z_max;
  std::string image_sub_topic_name;
  std::string point_sub_topic_name;
  std::string speed_sub_topic_name;
  float intensity_scale;
  tf2::Quaternion myQuaternion;
  std::string pp_onnx_file_;
  PointPillar *pointpillar;
  Yolov5 *yolov5;
  //json save event  
  Json::Value root;
  //json save adas warning
  Json::Value warning;
  //json save speed warning
  Json::Value speed_warning;
  //json save can messages
  Json::Value can_pub;
  //camera matrix
  /*float calib[3][4]={7.5252571368897372e+02, 0, 3.1557279173419107e+02, 0.000000000000e+00,
  0.000000000000e+00,  1.0017426964239700e+03, 2.6525972672211282e+02, 0.000000000000e+00,
  0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00};*/
  
  float calib[3][4]={4.6532098249027223e+02, 0, 3.2229066007638892e+02, 0.000000000000e+00,
  0.000000000000e+00,  6.2563849394337012e+02, 2.3709924767984074e+02, 0.000000000000e+00,
  0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00};
  
  //传感器外参标定矩阵
  Eigen::Matrix4d lidar_to_camera_matrix;
  
  //float v_car=10.0;
  float v_people=0.2;
  float a_stop=5;

  cv::VideoWriter writer;
  int coder=VideoWriter::fourcc('X','V','I','D');//选择编码格式
  double fps=10.0;//设置视频帧率
  
  std::vector<vector<save_message>> event_vector; //容器1嵌套容器2，容器2中存放自建结构体数据
  int count=0;
  cudaEvent_t start, stop;
  
  // Here is defined the synchronization policy, in this case is the approximate time
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2,custom_interfaces::msg::Stampstring> MySyncPolicy;
  // The synchronizer based on the chosen policy
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync;
  message_filters::Subscriber<sensor_msgs::msg::Image> imagesubfilter;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointsubfilter;
  message_filters::Subscriber<custom_interfaces::msg::Stampstring> speedsubfilter;

  float compute_iou(float bbox_2d[4] , float bbox_3d[4])
  {
     float x1=max(bbox_2d[0],bbox_3d[0]);
     float y1=max(bbox_2d[1],bbox_3d[1]);
     float x2=min(bbox_2d[2],bbox_3d[2]);
     float y2=min(bbox_2d[3],bbox_3d[3]);
     if (x2<=x1 || y2<=y1)
         return 0;
     float union_area=(x2-x1) * (y2-y1);
     float area1=(bbox_2d[2]-bbox_2d[0])*(bbox_2d[3]-bbox_2d[1]);
     float area2=(bbox_3d[2]-bbox_3d[0])*(bbox_3d[3]-bbox_3d[1]);
     float iou=union_area/(area1+area2-union_area);
     return iou;
  }
  void compute_box_3d(float dim[3], float location[3], float rotation_y, float coordinates[][3])
  {
    // dim: 3
    // location: 3
    // rotation_y: 1
    // coordinates 8 x 3
    float c = cos(rotation_y); 
    float s = sin(rotation_y);
    float R[3][3] = {c, 0, s, 0, 1, 0, -s, 0, c};
    float l = dim[2];
    float w = dim[1];
    float h = dim[0];

    float coordinates_ori[8][3] = {l/2, h/2, w/2, l/2, h/2, -w/2, -l/2, h/2, -w/2, -l/2, h/2, w/2, l/2, -h/2, w/2, l/2, -h/2, -w/2, -l/2, -h/2, -w/2, -l/2, -h/2, w/2};
    for (int m = 0; m < 8; m++)
      {
        for (int n = 0; n < 3; n++)
        {
            coordinates[m][n] = 0;
            for (int k = 0; k < 3; k++)
            {
                coordinates[m][n] += coordinates_ori[m][k] * R[k][n];
            }
        }
      }
    for (int i = 0; i < 8; i++) 
      {	
        coordinates[i][0] +=  location[0];
        coordinates[i][1] +=  location[1];
        coordinates[i][2] +=  location[2];
	    }
  }


  void project_to_image(float pts_3d[][8],float P[][4], float pts_2d[][2])
  {
    // pts_3d: 4 x 8
    // P: 3 x 4
    // pts_2d: 8 x 2
    float pts_2d_temp[3][8];
    for (int m = 0; m < 3; m++)
      {
        for (int n = 0; n < 8; n++)
        {
            pts_2d_temp[m][n] = 0;
            for (int k = 0; k < 4; k++)
            {
                pts_2d_temp[m][n] +=  P[m][k] * pts_3d[k][n];
            }
        }
      }
    for (int i = 0; i < 8; i++) 
      {	
        pts_2d[i][0] =  pts_2d_temp[0][i]/pts_2d_temp[2][i];
        pts_2d[i][1] =  pts_2d_temp[1][i]/pts_2d_temp[2][i];
	    }    
  }

  bool AllisNum(string str)  
  {  
      for (int i = 0; i < str.size(); i++)
      {
          int tmp = (int)str[i];
          if (tmp >= 48 && tmp <= 57)
          {
              continue;
          }
          else
          {
              return false;
          }
      } 
      return true;
  }
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg1,const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg2,const custom_interfaces::msg::Stampstring::ConstSharedPtr msg3)
  {
    time_t t = time(0); 
    char tmp[32]={NULL};
    char date[32]={NULL};
    char time[32]={NULL};
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&t));
    strftime(date, sizeof(date), "%Y-%m-%d",localtime(&t));
    strftime(time, sizeof(time), "%H:%M:%S",localtime(&t));
    std::string date_now=date;
    std::string time_now=time;

    //create video_filepath and image_filepath 
    string video_filepath="/home/nvidia/datalog/adas/video/"+date_now;
    string image_filepath="/home/nvidia/datalog/adas/ftp/image/"+date_now;
    string command_video;
    string command_image;

    //create system_log and alert_log path
    string status_filepath="/home/nvidia/datalog/adas/system_log/"+date_now; //date_ come from launch.py
    string alerts_filepath="/home/nvidia/datalog/adas/alert_log/"+date_now;
    string speed_limit_filepath="/home/nvidia/datalog/speed_limit/"+date_now;
    string command_alerts;
    string command_speed_limit;
    string status_filename=status_filepath+"/"+"status.txt"; //path had been created in pointpillar.cpp
    
    if (0 != access(video_filepath.c_str(), 0))
    {
      // 返回 0 表示创建成功，-1 表示失败
      command_video = "mkdir -p " + video_filepath;
      system(command_video.c_str());    	
    }

    if (0 != access(image_filepath.c_str(), 0))
    {
      // 返回 0 表示创建成功，-1 表示失败
      command_image = "mkdir -p " + image_filepath;
      system(command_image.c_str());    	
    }  

    if (0 != access(alerts_filepath.c_str(), 0))
      {
        // 返回 0 表示创建成功，-1 表示失败
        command_alerts = "mkdir -p " + alerts_filepath;
        system(command_alerts.c_str());    	
      }

    if (0 != access(speed_limit_filepath.c_str(), 0))
      {
        // 返回 0 表示创建成功，-1 表示失败
        command_speed_limit = "mkdir -p " + speed_limit_filepath;
        system(command_speed_limit.c_str());    	
      }

    can_pub["adas"] = Json::Value("adas");
    //sensor status check
    std::ofstream ofs;
    ofs.open(status_filename,std::ios::app);
    if (msg1->width !=0){ // only save once
      can_pub["4"] = Json::Value(0);
      if (pic_flag){
        if (ofs.is_open()){
            ofs << "  Adas camera works in normal condition.\n";
            //ofs << "\n";
        }
        pic_flag=false;
        }
      }
    else
      {can_pub["4"] = Json::Value(1);

       if (ofs.is_open()){
           ofs << time_now << ":";
           ofs << "  Adas camera works in error condition.\n";}   

        }

    if (msg2->width !=0){ // only save once
      can_pub["5"] = Json::Value(0);
      if (point_flag){

        if (ofs.is_open()){
            ofs << "  Laser works in normal condition.\n";
            //ofs << "\n";
          }
        point_flag=false;
        
        }
      }
    else
      {can_pub["5"] = Json::Value(1);

       if (ofs.is_open()){
           ofs << time_now << ":";
           ofs << "  Laser camera works in error condition.\n";}
        }
     ofs.close();
    //传感器外参标定矩阵
    //Eigen::Matrix4d lidar_to_camera_matrix;
    
    /*lidar_to_camera_matrix << -9.9204360006986247e-01, -1.2508047566197344e-01,
       1.4295809476907784e-02, 3.4371959250245221e-02,
       5.7338251921911637e-04, -1.1804145090421692e-01,
       -9.9300850303555477e-01, 2.8926625337878493e-02,
       1.2589347398857589e-01, -9.8509953328412514e-01,
       1.1717398487072994e-01, -1.8790705587862949e-01, 
       0.0, 0.0, 0.0, 1.0;*/
    cv::Mat cv_cam_matrix_=(cv::Mat_<double>(3, 3) << 4.6532098249027223e+02, 0, 3.2229066007638892e+02, 0, 6.2563849394337012e+02, 2.3709924767984074e+02, 0, 0, 1);
    cv::Mat cv_dist_params_ = (cv::Mat_<float>(5, 1) << -0.140819,0.111515,0.000165726,-0.000272997,-0.0291772);
    //计算安全距离和制动距离
    float v_car=msg3->speed;
    float t0=v_car/a_stop;
    float braking_distance=v_car*v_car/(2*a_stop);
    float safety_distance=15;

   //使能开关，控制检测程序是否执行
    int ch;
    int temp;
    ifstream enable_dirStream("/sys/class/gpio/PA.02/value");
    ifstream pass_dirStream("/sys/class/gpio/PG.07/value");
    int enable_value,pass_value;
    enable_dirStream>>enable_value;
    pass_dirStream>>pass_value;


    //yolov5 模型检测及输出
    cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::TYPE_8UC3);
    Mat out_img;
    Mat distort_out_img;
    vector<_bbox> bbox;

    
    if(enable_value==1){ //value为1，不执行
      //frame[1].data[0] =0x00;
      can_pub["1"]=Json::Value(1); 
      std::cerr<<"****ch value***"<<enable_value<<std::endl;
      bbox.clear();
      //sleep(3);
      }      
    else {  //value为0，执行 
      //frame[1].data[0] =0x01;   
      can_pub["1"]=Json::Value(0); 
      yolov5->objDetection(cv_ptr_img->image, out_img, bbox);}
    cv::undistort(out_img, distort_out_img, cv_cam_matrix_, cv_dist_params_, noArray());
    if(pass_value==1){ //value为1，不执行
      //frame[1].data[1] =0x00;
      can_pub["2"]=Json::Value(1); 
      }   
    else {//value为0，执行 
      //frame[1].data[1] =0x01; 
      can_pub["2"]=Json::Value(0); 
      }    
    //nbytes1 = write(s, &frame[1],sizeof(frame[1]));//发送 frame[1]
    can_pub["0"]=Json::Value(90);
    //智能限速模块
    if (bbox.size()!=0){
    for (auto obj : bbox){
      string label_name = obj.class_id;
      if(AllisNum(label_name)){
        std::cerr<<"yolo label: "<<label_name<<std::endl;
        int v_car_max=atoi(label_name.c_str());
        if(v_car > v_car_max){
          //通过CAN下发减速指令
          //to be done
          //json save speed warning
          //can_pub["0"]=Json::Value(90);
          //frame[1].data[0] =0x10; 
          //ros2 json speed warning message 
          speed_warning["time"]=Json::Value(time_now);
          speed_warning["car number"]=Json::Value("jiaolunche 1");
          speed_warning["car speed"]=Json::Value(v_car);
          speed_warning["warning type"]=Json::Value("chao su");
          speed_warning["speed limit"]=Json::Value(v_car_max);
          //publish warning message 
          auto string_speed_warn_msg = std_msgs::msg::String();
          Json::FastWriter sw;
          string_speed_warn_msg.data=sw.write(speed_warning);
          string_speed_pub->publish(string_speed_warn_msg);
          //warning message logs 
          std::ofstream ofs;
          string speed_limit_filepath_name=speed_limit_filepath+"/"+"speed_alerts.txt";
          ofs.open(speed_limit_filepath_name,std::ios::app);
          if (ofs.is_open()){
              ofs << "time: " << time_now << " ";
              ofs << "car number: " << "jiaolunche 1" << " ";
              ofs << "car speed: " << v_car << " ";
              ofs << "warning type: " <<"chao su" << " ";
              ofs << "brake ratio: " <<90 << " ";
              ofs << "speed limit: " << v_car_max << "\n";
          }
          ofs.close();
          }
        }
      }
    }
    
    //PointPillars 模型检测及输出
    assert(data_type == "fp32" || data_type == "fp16");
    
    float elapsedTime = 0.0f;
    cudaStream_t stream = NULL;
    
    vector<save_message> time_event_list; //容器，存放自建结构体数据，参考save_message结构体定义
    list<string> event_list; //列表，存放事件（event）
    std::string event_id;  //事件id

    checkCudaErrors(cudaEventCreate(&start));
    checkCudaErrors(cudaEventCreate(&stop));
    checkCudaErrors(cudaStreamCreate(&stream));
    
    Params params_;
    std::vector<Bndbox> nms_pred; //存放检测结构
    nms_pred.reserve(100);


    //PointPillar pointpillar(pp_onnx_file_, stream);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg2, *pcl_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // 直通滤波

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min,x_max);
    //保留下的数据索引
    std::vector<int> indices_x;
    pass.filter(indices_x);
    //获取过滤掉的点的索引
    pcl::IndicesConstPtr indices_rem = pass.getRemovedIndices();
    pcl::IndicesPtr index_ptr_x = boost::make_shared<std::vector<int>>(indices_x);
    //将保留下来的点的索引代替输入的数据集
    pass.setIndices(index_ptr_x);
    
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min,y_max);
    //保留下的数据索引
    std::vector<int> indices_y;
    pass.filter(indices_y);
    pcl::IndicesPtr index_ptr_y = boost::make_shared<std::vector<int>>(indices_y);
    pass.setIndices(index_ptr_y);
    
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min,z_max);
    // pass.setFilterLimitsNegative (true);   // 对设置的范围取反，本来是对设置的范围内的点云可以通过的，现在取反就是设置的范围内的点云不能通过
    pass.filter(*cloud_filtered);
    
    unsigned int num_point_values = cloud_filtered->size();

    unsigned int points_size = cloud_filtered->points.size();

    std::vector<float> pcl_data;

    for (const auto& point : cloud_filtered->points){
      pcl_data.push_back(point.x);
      pcl_data.push_back(point.y);
      pcl_data.push_back(point.z);
      pcl_data.push_back(point.intensity/intensity_scale);
      }

    float* points = static_cast<float *>(pcl_data.data());
    
    //Use 4 because PCL has padding (4th value now has intensity information)
    unsigned int points_data_size = points_size * sizeof(float) * 4;

    float *points_data = nullptr;

    checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
    checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
    checkCudaErrors(cudaDeviceSynchronize());
    cudaEventRecord(start, stream);

    if(enable_value==1){ //value为1，不执行
      std::cerr<<"****ch value***"<<enable_value<<std::endl;
      nms_pred.clear();
      //sleep(3);
      } 
    else {   //value为0，执行    
    temp=pointpillar->doinfer(points_data, points_size, nms_pred, nms_threshold, score_threshold);
      }
    
    if (temp == 0){
      if (nms_pred.size()!=0){
      for(int i=0; i<nms_pred.size(); i++) {
        int class_id = (int)nms_pred[i].id;
        std::string class_names=params_.class_name[class_id];
        float object_distance=sqrt((nms_pred[i].x)*(nms_pred[i].x)+
                             (nms_pred[i].y)*(nms_pred[i].y)+
                             (nms_pred[i].z)*(nms_pred[i].z));

	geometry_msgs::msg::Quaternion orientation;
        myQuaternion.setRPY(0, 0, nms_pred[i].rt);
        orientation = tf2::toMsg(myQuaternion);
        std::cerr<<"object distance: "<<object_distance<<std::endl;
        float rota_y=atan2(2*(orientation.w*orientation.x+orientation.y*orientation.z),(1-2*(orientation.x*orientation.x+orientation.y*orientation.y)));
        
        float dim[3] = {nms_pred[i].h,nms_pred[i].w,nms_pred[i].l};
        float point_3d_8[8][3];
        clock_t start_cpu, finish_cpu;

        Eigen::Vector4d lidar_position(nms_pred[i].x,nms_pred[i].y,nms_pred[i].z,1.0);
        //Eigen::Vector4d lidar_in_camera_position=lidar_to_camera_matrix.colPivHouseholderQr().solve(lidar_position);
        Eigen::Vector4d lidar_in_camera_position=lidar_to_camera_matrix.inverse()*lidar_position;
        float location[3] = {lidar_in_camera_position[0],lidar_in_camera_position[1],lidar_in_camera_position[2]};
        compute_box_3d(dim, location, rota_y, point_3d_8);

        float point_3d_8_temp[4][8];
          for (int m = 0; m < 8; m++)
          {
            for (int n = 0; n < 3; n++)
            {
                  point_3d_8_temp[n][m] = point_3d_8[m][n];
              }
              point_3d_8_temp[3][m] = 0.0;
          }
         
        float pts_2d[8][2];
        float xy_min[2];
        float xy_max[2];
        project_to_image(point_3d_8_temp, calib, pts_2d);
        
        for(int i=0;i<2;i++){
            xy_min[i]=pts_2d[0][i];
            for(int j=0;j<8;j++){
                if(pts_2d[j][i]<xy_min[i]){
                    xy_min[i]=pts_2d[j][i];}
                }
            }
        for(int i=0;i<2;i++){
            xy_max[i]=pts_2d[0][i];
            for(int j=0;j<8;j++){
                if (pts_2d[j][i]>xy_max[i]){
                    xy_max[i]=pts_2d[j][i];}      
                }
            }
        
        
        for (auto obj : bbox){
          float bbox_2d[4]={obj.xmin,obj.ymin,obj.xmax,obj.ymax};
          //float iou; 
          float bbox_3d[4]={xy_min[0],xy_min[1],xy_max[0],xy_max[1]};
          int class_id = atoi(obj.class_id.c_str());
          cv::Scalar color = cv::Scalar(coco::color_list[class_id][0],
                                      coco::color_list[class_id][1],
                                      coco::color_list[class_id][2]);
          Point p1(obj.xmin,obj.ymin),p2(obj.xmax,obj.ymax);                   
          //float iou=compute_iou(bbox_2d,bbox_3d);
  
          //if (iou>0.3){
            rectangle(distort_out_img, p1,p2, color * 255, 2);
            putText(distort_out_img, obj.class_id,
            Point(obj.xmin, obj.ymin - 1), FONT_HERSHEY_PLAIN, 1.2,
            Scalar(0xFF, 0xFF, 0xFF), 2);
            //}       
          }

  	 Point p3(xy_min[0],xy_min[1]),p4(xy_max[0],xy_max[1]);
        Scalar colorRectangle(0,0,255); 
        rectangle(distort_out_img, p3,p4, colorRectangle, 2);
        string pp_message=class_names+", "+std::to_string(object_distance);
        putText(distort_out_img, pp_message,
            Point(xy_min[0],xy_min[1] - 1), FONT_HERSHEY_PLAIN, 1.2,
            colorRectangle, 2);

        
        if (object_distance >= 15)
            {event_id="10000001";
             std::cerr<<"提示"<<std::endl;}
        else if (object_distance >= 8 && object_distance < 15)
            {event_id="10000002";
             std::cerr<<"预警"<<std::endl;}
        else if (object_distance >= 3 && object_distance < 8)
            {event_id="10000003";
             std::cerr<<"报警"<<std::endl;}
        else if (object_distance <3)
            {event_id="10000004";
             std::cerr<<"制动"<<std::endl;}
        event_list.emplace_back(event_id);  
            
      } //for 循环 结束
      event_list.sort();
      //std::cerr<<"event_list0 "<<event_list.size()<<std::endl;
      event_id=event_list.back();
      //std::cerr<<"event_id "<<event_id<<std::endl;  

      save_message tmp_save_message;
      tmp_save_message.tmp=tmp;
      tmp_save_message.time_=time_now;
      tmp_save_message.event_id=event_id;
      tmp_save_message.out_img=distort_out_img;
      time_event_list.push_back(tmp_save_message);
      //time_event_list.push_back(tmp);
      //time_event_list.push_back(time);
      //time_event_list.push_back(event_id);
      std::cerr<<"time_event_list "<<time_event_list.size()<<std::endl;
      if (event_id!=""){
      event_vector.push_back(time_event_list);
      if(pass_value==1){ //value为1，发送报警信息
            //ros2 json CAN pub messgae
        can_pub["3"]=Json::Value(1);}
      else {   //value为0，不发送报警信息 
        can_pub["3"]=Json::Value(0);} }
      std::cerr<<"event_vector "<<event_vector.size()<<std::endl;

	    writer.write(distort_out_img);
	    count+=1;

      if (event_vector.size()>1)
      {
        vector<save_message> time_event_list_first=event_vector[0];
        vector<save_message> time_event_list_last=event_vector[event_vector.size()-2];
        vector<save_message> time_event_list_current=event_vector[event_vector.size()-1];
        string last_event_id=time_event_list_last[0].event_id;
        string current_event_id=time_event_list_current[0].event_id;
        std::cerr<<"time_event_list_last "<<last_event_id<<std::endl;
        std::cerr<<"time_event_list_current "<<current_event_id<<std::endl;
        if (last_event_id != current_event_id)
        {

          string event_id=time_event_list_last[0].event_id;
          string current_time_=time_event_list_current[0].time_;

          string start_time=time_event_list_first[0].time_;
          Mat start_image=time_event_list_first[0].out_img;
          string end_time=time_event_list_last[0].time_;
          Mat end_image=time_event_list_last[0].out_img;

          string start_image_url="ftp://nvidia:nvidia@127.0.0.1/"+image_filepath+"/"+start_time+"-"+event_id+".jpg";
          string start_image_path=image_filepath+"/"+start_time+"-"+event_id+".jpg";
          string end_image_url="ftp://nvidia:nvidia@127.0.0.1/"+image_filepath+"/"+end_time+"-"+event_id+".jpg";
          string end_image_path=image_filepath+"/"+end_time+"-"+event_id+".jpg";
          //ros2 json event messgae
          root["start_time"] = Json::Value(start_time); 
          root["end_time"] = Json::Value(end_time); 
          root["event_code"]= Json::Value(event_id);
          root["begin_image_url"]= Json::Value(start_image_url);
          root["end_image_url"]= Json::Value(end_image_url);
          //publish event message
          auto string_event_msg = std_msgs::msg::String();
          Json::FastWriter ww;
          string_event_msg.data=ww.write(root);
          string_event_pub->publish(string_event_msg);

          //ros2 json warning message 
          warning["time"]=Json::Value(start_time);
          warning["car number"]=Json::Value("jiaolunche 1");
          warning["car speed"]=Json::Value(v_car);
          warning["warning type"]=Json::Value(event_id);
          //publish warning message 
          auto string_warn_msg = std_msgs::msg::String();
          Json::FastWriter ew;
          string_warn_msg.data=ew.write(warning);
          string_warn_pub->publish(string_warn_msg);
          //warning message logs 
          std::ofstream ofs;
          string alerts_filepath_name=alerts_filepath+"/"+"alerts.txt";
          ofs.open(alerts_filepath_name,std::ios::app);
          if (ofs.is_open()){
              ofs << "alert message: " << "\n";
              ofs << " time: " << start_time << " ";
              ofs << " car number: " << "jiaolunche 1" << " ";
              ofs << " car speed: " << v_car << " ";
              ofs << " warning type: " << event_id << "\n";
              
              ofs << "event message: " << "\n";
              ofs << " start_time: " << start_time << " ";
              ofs << " end_time: " << end_time << " ";
              ofs << " event_code: " << event_id << " ";
              ofs << " begin_image_url: " << start_image_url << " ";
              ofs << " end_image_url: " << end_image_url << "\n";
              
            }
          ofs.close();

          //出现事件时才像CAN发送报警信息//
          //超越开关，控制报警信息发送
          /*if(pass_value==1){ //value为1，发送报警信息
            //ros2 json CAN pub messgae
            can_pub["3"]=Json::Value(1);}
          else {   //value为0，不发送报警信息 
            can_pub["3"]=Json::Value(0);} 

          //publish CAN pub message
          auto string_can_msg = std_msgs::msg::String();
          Json::FastWriter cw;
          string_can_msg.data=cw.write(can_pub);
          string_can_pub->publish(string_can_msg); */

          //save start_image and end_image of event_id for ftp usage 
          imwrite(start_image_path,start_image);
          imwrite(end_image_path,end_image);

          //clear operation
          event_vector.clear();
          writer.release();
          //save video
          string filename=video_filepath+"/"+current_time_+".avi";//保存的视频文件名称
          writer.open(filename,coder,fps,Size(640,480),true);//创建保存视频文件的视频流

          }
        }

         /*if (count%100==0) //1s 10zhen ,10s 100zhen
         {
            writer.release();
            time_t t = time(0); 
  	        char date[32]={NULL};
  	        char time[32]={NULL};
            strftime(date, sizeof(date), "%Y-%m-%d",localtime(&t));
  	        strftime(time, sizeof(time), "%H:%M:%S",localtime(&t));
  	        std::string date_=date;
  	        std::string time_=time;
  	        string filepath="/home/nvidia/datalog/video/"+date_;
  	        string filename=filepath+"/"+time_+".avi";//保存的视频文件名称
            writer.open(filename,coder,fps,Size(640,480),true);//创建保存视频文件的视频流
            }*/
      }  
    } //if 结束  
    

    //publish topics
    sensor_msgs::msg::Image img_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", distort_out_img).toImageMsg(img_msg);
    img_msg.header.frame_id = "camera";
    
    fusion_image_pub->publish(img_msg);

    vector<uchar> jpeg_data;
    imencode(".jpg", out_img, jpeg_data);
    sensor_msgs::msg::CompressedImage compressed_msg;
    compressed_msg.format = "jpeg";
    compressed_msg.data = jpeg_data;
    compressed_publisher->publish(compressed_msg);     

    //publish CAN pub message
     auto string_can_msg = custom_interfaces::msg::Stampstring();
     auto stamp = this->get_clock()->now();
     string_can_msg.header.stamp = stamp;
     Json::FastWriter cw;
     string_can_msg.data=cw.write(can_pub);
     string_can_pub->publish(string_can_msg); 
     
    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);

    auto message = std_msgs::msg::String();
    message.data = "TIME: " + std::to_string(elapsedTime) + " ms, Objects detected: " + std::to_string(nms_pred.size());
    RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());

    checkCudaErrors(cudaFree(points_data));
    //checkCudaErrors(cudaFree(points_num));
    nms_pred.clear();
    root.clear();
    event_list.clear();
    time_event_list.clear();
    can_pub.clear();
    //close(s);

  checkCudaErrors(cudaEventDestroy(start));
  checkCudaErrors(cudaEventDestroy(stop));
  checkCudaErrors(cudaStreamDestroy(stream));


  }  //回调函数 结束


  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fusion_image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_event_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_warn_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_speed_pub;
  rclcpp::Publisher<custom_interfaces::msg::Stampstring>::SharedPtr string_can_pub;

 };

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointPillarsROS>());
  rclcpp::shutdown();
  return 0;
}
