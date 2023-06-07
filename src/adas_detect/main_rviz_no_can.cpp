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
#include <unistd.h>
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
#include <visualization_msgs/msg/marker_array.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.h>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <unistd.h>
#include <sys/stat.h>
//yolov5
//#include "detection_node.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "yolov5.h"
#include "yoloparam.h"
//#include "target_bbox_msgs/msg/bounding_box.hpp"
//#include "target_bbox_msgs/msg/bounding_boxes.hpp"

//message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
/*报文发送程序*/
#include <stdio.h>
#include <stdlib.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define CLOCKS_PER_SEC ((clock_t)1000)

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

class PointPillarsROS : public rclcpp::Node
{
public: 
  yoloparam YP;
  void* YParam;

  PointPillarsROS()
  : Node("pointpillars_ros")
  {

  //algorithm pointpillar related params
  this->declare_parameter<std::string>("date", "date");
  this->declare_parameter<std::string>("time", "time");
  this->declare_parameter<std::string>("data_type", "fp16");
  this->declare_parameter<string>("point_sub_topic_name", "/lslidar_point_cloud");
  this->declare_parameter<float>("intensity_scale", 1.0);
  this->declare_parameter<float>("score_threshold", 0.4);
  this->declare_parameter<float>("nms_pp_threshold", 0.01);
  this->declare_parameter<std::string>("pointpillar_onnx_file", "../model/pointpillar.onnx");
  

  //algorithm yolov5 related params
  this->declare_parameter<string>("image_sub_topic_name", "/adas_video_frames");
  this->declare_parameter<int>("device", 0);
  this->declare_parameter<double>("nms_yolo_threshold", 0.45);
  this->declare_parameter<double>("conf", 0.5);
  this->declare_parameter<int>("batch_size", 1);
  this->declare_parameter<int>("input_h", 640);
  this->declare_parameter<int>("input_w", 640);
  this->declare_parameter<int>("class_num", 68);
  this->declare_parameter<string>("engine_dir", "../engine/best.engine");

  
  //pointpillar
  date_=this->get_parameter("date").as_string();
  time_=this->get_parameter("time").as_string();
  data_type = this->get_parameter("data_type").as_string();
  std::string point_sub_topic_name=this->get_parameter("point_sub_topic_name").as_string();
  intensity_scale = this->get_parameter("intensity_scale").as_double();
  score_threshold=this->get_parameter("score_threshold").as_double();
  nms_threshold=this->get_parameter("nms_pp_threshold").as_double();
  pp_onnx_file_=this->get_parameter("pointpillar_onnx_file").as_string();
  RCLCPP_INFO(this->get_logger(),"init start %s", pp_onnx_file_.c_str());
  std::cerr<<"score_threshold "<<score_threshold<<std::endl;
  
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
  RCLCPP_INFO(this->get_logger(), "class_num: '%d'", YP.CLASS_NUM);
  YParam = &YP;
  
  cudaStream_t stream = NULL;
  pointpillar= new PointPillar(pp_onnx_file_, stream);

  
  //PointPillar pointpillar(pp_onnx_file_, stream);
  yolov5 = new Yolov5(YParam);
  /*auto nh=std::make_shared<PointPillarsROS>();
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(nh.get(),image_sub_topic_name);
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_sub(nh.get(),point_sub_topic_name);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> testSyncPolicy;
  message_filters::Synchronizer<testSyncPolicy> syncApproximate(testSyncPolicy(10), image_sub, point_sub);
  syncApproximate.registerCallback(std::bind(&PointPillarsROS::topic_callback, _1, _2));*/
  //cv::VideoWriter writer;
  
  int coder=VideoWriter::fourcc('M','J','P','G');//选择编码格式
  double fps=10.0;//设置视频帧率

  string video_filepath="/home/nvidia/datalog/adas/video/"+date_;
  string image_filepath="/home/nvidia/datalog/adas/ftp/image/"+date_;
  string command_video;
  string command_image;
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


  string filename=video_filepath+"/"+time_+".avi";//保存的视频文件名称
  writer.open(filename,coder,fps,Size(640,480),true);//创建保存视频文件的视频流
  

  imagesubfilter.subscribe(this,image_sub_topic_name);
  pointsubfilter.subscribe(this,point_sub_topic_name);
  sync.reset(new Sync(MySyncPolicy(10),imagesubfilter,pointsubfilter));
  sync->registerCallback(std::bind(&PointPillarsROS::topic_callback, 
               this, _1, _2));

  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bbox", 700);
  publisher_text = this->create_publisher<visualization_msgs::msg::MarkerArray>("text", 700);
  
  fusion_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
      "/adas/fusion_obj_detection", 3);
  string_event_pub= this->create_publisher<std_msgs::msg::String>("/event_messages", 700);
  string_warn_pub= this->create_publisher<std_msgs::msg::String>("/warn_messages", 700);




  }
  


private:

  const int NUM_POINT_FEATURE_=5;
  const int OUTPUT_NUM_BOX_FEATURE_=7;
  int pic_flag=0;
  int point_flag=0;

  std::string data_type;
  std::string date_;
  std::string time_;
  float nms_threshold;
  float score_threshold;
  std::string image_sub_topic_name;
  std::string point_sub_topic_name;
  float intensity_scale;
  tf2::Quaternion myQuaternion;
  std::string pp_onnx_file_;
  PointPillar *pointpillar;
  //std::shared_ptr<PointPillar> pointpillar;
  
  //cudaStream_t stream = NULL;
  Yolov5 *yolov5;
  //节点存储event  
  Json::Value root;
  //节点存储car can message
  Json::Value warning;
  float calib[3][4]={4.6532098249027223e+02, 0, 3.2229066007638892e+02, 0.000000000000e+00,
  0.000000000000e+00,  6.2563849394337012e+02, 2.3709924767984074e+02, 0.000000000000e+00,
  0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00};
  float v_car=10.0;
  float v_people=0.2;
  float a_stop=5;
  cv::VideoWriter writer;
  int coder=VideoWriter::fourcc('M','J','P','G');//选择编码格式
  double fps=10.0;//设置视频帧率
  
  std::vector<vector<save_message>> event_vector;

  int count=0;
  cudaEvent_t start, stop;
  
  // Here is defined the synchronization policy, in this case is the approximate time
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> MySyncPolicy;
  // The synchronizer based on the chosen policy
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync;
  message_filters::Subscriber<sensor_msgs::msg::Image> imagesubfilter;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointsubfilter;


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
    for (int i = 0; i < 8; i++) 
    {
        std::cout<<"i: "<<i<<" x: "<<pts_2d[i][0]<<" y: "<< pts_2d[i][1]<< std::endl;
    }  
    std::cout<<std::endl;
}


  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg1,const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {

    string status_filepath="/home/nvidia/datalog/adas/system_log/"+date_; 
    string alerts_filepath="/home/nvidia/datalog/adas/alert_log/"+date_;
    string command_status;
    string command_alerts;
    if (0 != access(status_filepath.c_str(), 0))
    {
        // 返回 0 表示创建成功，-1 表示失败
        command_status = "mkdir -p " + status_filepath;
        system(command_status.c_str());    	
    }

    if (0 != access(alerts_filepath.c_str(), 0))
    {
        // 返回 0 表示创建成功，-1 表示失败
        command_alerts = "mkdir -p " + alerts_filepath;
        system(command_alerts.c_str());    	
    }

    if (msg1!=0){
      if (pic_flag==0){
        std::ofstream ofs;
        ofs.open(status_filepath,std::ios::app);
        if (ofs.is_open()){
            ofs << "Adas camera works in normal condition.\n";
            //ofs << "\n";
        }
        pic_flag=1;
        //ofs.close();
      }
    }

    if (msg!=0){
      if (point_flag==0){
        std::ofstream ofs;
        ofs.open(status_filepath,std::ios::app);
        if (ofs.is_open()){
            ofs << "Laser works in normal condition.\n";
            //ofs << "\n";
        }
        point_flag=1;
        //ofs.close();
      }
    }
    
    

    Eigen::Matrix4d lidar_to_camera_matrix;
        lidar_to_camera_matrix << -0.992,-0.125,0.0143,0.0344,
  0.000573,-0.118041450904217,-0.993,0.0289,
  0.126,-0.985099533284125,0.117,-0.188,
  0.0, 0.0, 0.0, 1.0;
  float t0=v_car/a_stop;
  float braking_distance=v_car*v_car/(2*a_stop);
  float safety_distance=v_car*1+v_people*(t0)+braking_distance;
   
  cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::TYPE_8UC3);
  //cvtColor(cv_ptr_img->image, cv_ptr_img->image, COLOR_BGR2RGB);
  Mat out_img;
  vector<_bbox> bbox;
  yolov5->objDetection(cv_ptr_img->image, out_img, bbox); //Box
  std::cerr<<"yolo bbox size "<<std::to_string(bbox.size())<<std::endl;
  //target_bbox_msgs::msg::BoundingBoxes boxes;
  //boxes.header = cv_ptr_img->header;
  
    assert(data_type == "fp32" || data_type == "fp16");
    
    float elapsedTime = 0.0f;
    cudaStream_t stream = NULL;
    
    string video_filepath;
    string image_filepath;
    //vector<string> time_event_list;
    vector<save_message> time_event_list;
    list<string> event_list;
    std::string event_id;
    checkCudaErrors(cudaEventCreate(&start));
    checkCudaErrors(cudaEventCreate(&stop));
    checkCudaErrors(cudaStreamCreate(&stream));
    
    Params params_;
    std::vector<Bndbox> nms_pred;
    nms_pred.reserve(100);


    //PointPillar pointpillar(pp_onnx_file_, stream);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

      unsigned int num_point_values = pcl_cloud->size();

      unsigned int points_size = pcl_cloud->points.size();

      std::vector<float> pcl_data;

      for (const auto& point : pcl_cloud->points) {
        pcl_data.push_back(point.x);
        pcl_data.push_back(point.y);
        pcl_data.push_back(point.z);
        pcl_data.push_back(point.intensity/intensity_scale);
      }
      float* points = static_cast<float *>(pcl_data.data());
      
      //Use 4 because PCL has padding (4th value now has intensity information)
      unsigned int points_data_size = points_size * sizeof(float) * 4;


      float *points_data = nullptr;
      //unsigned int *points_num = nullptr;
      //unsigned int points_data_size = points_size * num_point_values * sizeof(float);
      checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
      //checkCudaErrors(cudaMallocManaged((void **)&points_num, sizeof(unsigned int)));
      checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
      //checkCudaErrors(cudaMemcpy(points_num, &points_size, sizeof(unsigned int), cudaMemcpyDefault));
      checkCudaErrors(cudaDeviceSynchronize());
      
      cudaEventRecord(start, stream);
      //clock_t started_cpu, finished_cpu;
      //started_cpu = clock();
      
      int temp=pointpillar->doinfer(points_data, points_size, nms_pred, nms_threshold, score_threshold);

      //finished_cpu = clock();
      //double duration = (double)(finished_cpu - started_cpu) / CLOCKS_PER_SEC;//duration是CPU的计算时间
      //std::cerr<<"total calback seconds: "<<std::to_string(duration)<<std::endl;
      
      visualization_msgs::msg::MarkerArray detections;
      visualization_msgs::msg::MarkerArray texts;
      //std::cerr<<"points_num size"<<std::to_string(points_num)<<std::endl;
      std::cerr<<"nms_pred size"<<std::to_string(nms_pred.size())<<std::endl;
      if (temp == 0){
      for(int i=0; i<nms_pred.size(); i++) {
        visualization_msgs::msg::Marker detection;
        visualization_msgs::msg::Marker text;
        text.lifetime=rclcpp::Duration::from_seconds(0.1);
        text.id=i;
        text.header.frame_id="/laser_link";
        text.ns = "text_shapes";
        text.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.6;
        text.color.b = 0;
        text.color.g = 1;
        text.color.r = 1;
        text.color.a = 0.6;
        text.pose.position.x = nms_pred[i].x;
        text.pose.position.y = nms_pred[i].y;
        text.pose.position.z = nms_pred[i].z+1;
        int class_id = (int)nms_pred[i].id;
        std::string class_names=params_.class_name[class_id];
        float object_distance=sqrt((nms_pred[i].x)*(nms_pred[i].x)+
                             (nms_pred[i].y)*(nms_pred[i].y)+
                             (nms_pred[i].z)*(nms_pred[i].z));
        //object_distance=round(object_distance*100)/100;
        //std::cerr<<"object_distance: "<<std::to_string(object_distance)<<std::endl;
        std::cerr<<"object_distance: "<<setiosflags(ios::fixed)<<setprecision(2)<<object_distance<<std::endl;
        std::string text_message=class_names+","+std::to_string(nms_pred[i].score)+"\n"+std::to_string(object_distance)+"m";
        text.text=text_message;
        texts.markers.emplace_back(text);
        
        
        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        geometry_msgs::msg::Pose center;
        geometry_msgs::msg::Vector3 size;
        geometry_msgs::msg::Point position;	
	 geometry_msgs::msg::Quaternion orientation;
        
        detection.id=i;
        detection.lifetime=rclcpp::Duration::from_seconds(0.1);
        detection.ns = "basic_shapes";
        detection.header.frame_id="/laser_link";
        detection.type=visualization_msgs::msg::Marker::CUBE;
        detection.action = visualization_msgs::msg::Marker::ADD;
        
        detection.pose.position.x = nms_pred[i].x;
        detection.pose.position.y = nms_pred[i].y;
        detection.pose.position.z = nms_pred[i].z;

        detection.scale.x = nms_pred[i].w;
        detection.scale.y = nms_pred[i].l;
        detection.scale.z = nms_pred[i].h;
        
        myQuaternion.setRPY(0, 0, nms_pred[i].rt);
        orientation = tf2::toMsg(myQuaternion);

        detection.pose.orientation.x = orientation.x;
        detection.pose.orientation.y = orientation.y;
        detection.pose.orientation.z = orientation.z;
        detection.pose.orientation.w = orientation.w;
        float rota_y=atan2(2*(orientation.w*orientation.x+orientation.y*orientation.z),(1-2*(orientation.x*orientation.x+orientation.y*orientation.y)));
        detection.color.r=1.0;
        detection.color.g=0.0;
        detection.color.b=0.0;
        detection.color.a=0.3;

        hyp.id = std::to_string(nms_pred[i].id);
        hyp.score = nms_pred[i].score;
        
        float dim[3] = {nms_pred[i].h,nms_pred[i].w,nms_pred[i].l};
        float point_3d_8[8][3];
        clock_t start_cpu, finish_cpu;
        start_cpu = clock();
 	    Eigen::Vector4d lidar_position(nms_pred[i].x,nms_pred[i].y,nms_pred[i].z,1.0);
	    Eigen::Vector4d lidar_in_camera_position=lidar_to_camera_matrix.colPivHouseholderQr().solve(lidar_position);
	    finish_cpu = clock();
	    double duration = (double)(finish_cpu - start_cpu) / CLOCKS_PER_SEC;//duration是CPU的计算时间
        std::cerr<<"matrix transport seconds: "<<std::to_string(duration)<<std::endl;
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
          
          //std::cerr<<"yolo class "<<obj.class_id<<std::endl;
                
	  //float iou=compute_iou(bbox_2d,bbox_3d);
	  //std::cerr<<"iou "<<std::to_string(iou)<<std::endl;
	  //if (iou>0.3)
	    //{  
	    rectangle(out_img, p1,p2, color * 255, 2);
               putText(out_img, obj.class_id,
               Point(obj.xmin, obj.ymin - 1), FONT_HERSHEY_PLAIN, 1.2,
                Scalar(0xFF, 0xFF, 0xFF), 2);
            //}
	  }
    		
  	
  	    Point p3(xy_min[0],xy_min[1]),p4(xy_max[0],xy_max[1]);
        Scalar colorRectangle(0,0,255); 
        rectangle(out_img, p3,p4, colorRectangle, 2);
        
        detections.markers.emplace_back(detection);
        
	
        if (object_distance > safety_distance)
            {event_id="10000001";
             std::cerr<<"提示"<<std::endl;}
        else if (object_distance > safety_distance-5 && object_distance <= safety_distance)
            {event_id="10000002";
             std::cerr<<"预警"<<std::endl;}
        else if (object_distance > braking_distance && object_distance < safety_distance-5)
            {event_id="10000003";
             std::cerr<<"报警"<<std::endl;}
        else if (object_distance >= braking_distance-5 && object_distance <braking_distance)
            {event_id="10000004";
             std::cerr<<"制动"<<std::endl;}
        event_list.emplace_back(event_id);  
            
      } //for 循环 结束
         event_list.sort();
         //std::cerr<<"event_list0 "<<event_list.size()<<std::endl;
         event_id=event_list.back();
         //std::cerr<<"event_id "<<event_id<<std::endl;
         
         time_t t = time(0); 
         char tmp[32]={NULL};
         char date[32]={NULL};
         char time[32]={NULL};
         strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&t));
         strftime(date, sizeof(date), "%Y-%m-%d",localtime(&t));
         strftime(time, sizeof(time), "%H:%M:%S",localtime(&t));
         std::string date_=date;
         std::string time_=time;

         video_filepath="/home/nvidia/datalog/adas/video/"+date_;
         image_filepath="/home/nvidia/datalog/adas/ftp/image/"+date_;
         string command_video;
         string command_image;
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

         save_message tmp_save_message;
         tmp_save_message.tmp=tmp;
         tmp_save_message.time_=time_;
         tmp_save_message.event_id=event_id;
         tmp_save_message.out_img=out_img;
         time_event_list.push_back(tmp_save_message);
         //time_event_list.push_back(tmp);
         //time_event_list.push_back(time);
         //time_event_list.push_back(event_id);
         std::cerr<<"time_event_list "<<time_event_list.size()<<std::endl;
         event_vector.push_back(time_event_list);
         std::cerr<<"event_vector "<<event_vector.size()<<std::endl;

	     writer.write(out_img);
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
                string start_time=time_event_list_first[0].tmp;
                Mat start_image=time_event_list_first[0].out_img;
                string end_time=time_event_list_last[0].tmp;
                string time_=time_event_list_current[0].time_;
                Mat end_image=time_event_list_last[0].out_img;
                string start_image_url="ftp://nvidia:nvidia@127.0.0.1/"+image_filepath+"/"+start_time+"-"+event_id+".jpg";
                string start_image_path=image_filepath+"/"+start_time+"-"+event_id+".jpg";
                string end_image_url="ftp://nvidia:nvidia@127.0.0.1/"+image_filepath+"/"+end_time+"-"+event_id+".jpg";
                string end_image_path=image_filepath+"/"+end_time+"-"+event_id+".jpg";
                root["start_time"] = Json::Value(start_time); 
                root["end_time"] = Json::Value(end_time); 
                root["event_code"]= Json::Value(event_id);
                root["begin_image_url"]= Json::Value(start_image_url);
                root["end_image_url"]= Json::Value(end_image_url);

                warning["time"]=Json::Value(start_time);
                warning["car number"]=Json::Value("jiaolunche 1");
                warning["car speed"]=Json::Value(v_car);
                warning["warning type"]=Json::Value(event_id);

                auto string_warn_msg = std_msgs::msg::String();
                Json::FastWriter ew;
                string_warn_msg.data=ew.write(warning);
                string_warn_pub->publish(string_warn_msg);


                std::ofstream ofs;
                string alerts_filepath_name=alerts_filepath+"/"+"alerts.txt";
                ofs.open(alerts_filepath_name,std::ios::app);
                if (ofs.is_open()){
                    ofs << "time" << start_time << " ";
                    ofs << "car number" << "jiaolunche 1" << " ";
                    ofs << "car speed" << v_car << " ";
                    ofs << "warning type" << event_id << "\n";


                }
                //ofs.close();

                auto string_event_msg = std_msgs::msg::String();
                Json::FastWriter ww;
                string_event_msg.data=ww.write(root);
                string_event_pub->publish(string_event_msg);

                imwrite(start_image_path,start_image);
                imwrite(end_image_path,end_image);
                event_vector.clear();

                writer.release();
                //string filepath="/home/nvidia/datalog/video/"+date_;
  	            string filename=video_filepath+"/"+time_+".avi";//保存的视频文件名称
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
        
      } //if 结束
      
      
      sensor_msgs::msg::Image img_msg;
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", out_img).toImageMsg(img_msg);
      img_msg.header.frame_id = "camera";
      
      fusion_image_pub->publish(img_msg);
      publisher_->publish(detections);
      publisher_text->publish(texts);
          
      
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


  checkCudaErrors(cudaEventDestroy(start));
  checkCudaErrors(cudaEventDestroy(stop));
  checkCudaErrors(cudaStreamDestroy(stream));

  }  //回调函数 结束
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_text;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fusion_image_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_event_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_warn_pub;
  size_t count_;

 
 };

void SaveBoxPred(std::vector<Bndbox> boxes, std::string file_name)
{
    std::ofstream ofs;
    ofs.open(file_name, std::ios::out);
    if (ofs.is_open()) {
        for (const auto box : boxes) {
          ofs << box.x << " ";
          ofs << box.y << " ";
          ofs << box.z << " ";
          ofs << box.w << " ";
          ofs << box.l << " ";
          ofs << box.h << " ";
          ofs << box.rt << " ";
          ofs << box.id << " ";
          ofs << box.score << " ";
          ofs << "\n";
        }
    }
    else {
      std::cerr << "Output file cannot be opened!" << std::endl;
    }
    ofs.close();
    std::cout << "Saved prediction in: " << file_name << std::endl;
    return;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointPillarsROS>());
  rclcpp::shutdown();
  return 0;
}
