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

#include <iostream>
#include <sstream>
#include <fstream>
#include <ctime>
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

//yolov5
//#include "detection_node.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "yolov5.h"
#include "yoloparam.h"
//#include "target_bbox_msgs/msg/bounding_box.hpp"
//#include "target_bbox_msgs/msg/bounding_boxes.hpp"

#define CLOCKS_PER_SEC ((clock_t)1000)
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;
using namespace cv;

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
  this->declare_parameter<float>("intensity_scale", 1.0);
  this->declare_parameter<float>("score_threshold", 0.5);
  this->declare_parameter<float>("nms_overlap_threshold", 0.01);
  this->declare_parameter<std::string>("pointpillar_onnx_file", "../model/pointpillar.onnx");
  
  //algorithm yolov5 related params
  this->declare_parameter<string>("image_sub_topic_name", "/adas_video_frames");
  this->declare_parameter<int>("device", 0);
  this->declare_parameter<double>("nms", 0.45);
  this->declare_parameter<double>("conf", 0.50);
  this->declare_parameter<int>("batch_size", 1);
  this->declare_parameter<int>("input_h", 640);
  this->declare_parameter<int>("input_w", 640);
  this->declare_parameter<int>("class_num", 80);
  this->declare_parameter<string>("engine_dir", "../engine/best.engine");

  
  //pointpillar
  rclcpp::Parameter data_type_param = this->get_parameter("data_type");
  data_type=data_type_param.as_string();
  intensity_scale = this->get_parameter("intensity_scale").as_double();
  score_threshold=this->get_parameter("score_threshold").as_double();
  nms_overlap_threshold=this->get_parameter("nms_overlap_threshold").as_double();
  std::string pp_onnx_file_=this->get_parameter("pointpillar_onnx_file").as_string();
  RCLCPP_INFO(this->get_logger(),"init start %s", pp_onnx_file_.c_str());
  
  //yolov5
  this->get_parameter("image_sub_topic_name", image_sub_topic_name);
  this->get_parameter("device", YP.DEVICE);
  this->get_parameter("nms", YP.NMS_THRESH);
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
  yolov5 = new Yolov5(YParam);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lslidar_point_cloud", 700, std::bind(&PointPillarsROS::topic_callback, this, _1));
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bbox", 700);
  publisher_text = this->create_publisher<visualization_msgs::msg::MarkerArray>("text", 700);
  
  obj_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
      "/image/obj_detection", 3);
  /*bboxs_pub = this->create_publisher<target_bbox_msgs::msg::BoundingBoxes>(
      "/targets/bboxs", 1000);*/
  image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      image_sub_topic_name, 3, std::bind(&PointPillarsROS::imageCallback, this, _1));
  
  }
  
  
  
  

private:

  const int NUM_POINT_FEATURE_=5;
  const int OUTPUT_NUM_BOX_FEATURE_=7;
  std::string data_type;
  std::string image_sub_topic_name;
  float intensity_scale;
  tf2::Quaternion myQuaternion;
  PointPillar* pointpillar;
  Yolov5 *yolov5;
  float score_threshold;
  float nms_overlap_threshold;
  void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    assert(data_type == "fp32" || data_type == "fp16");
    cudaEvent_t start, stop;
    float elapsedTime = 0.0f;
    cudaStream_t stream = NULL;

    checkCudaErrors(cudaEventCreate(&start));
    checkCudaErrors(cudaEventCreate(&stop));
    checkCudaErrors(cudaStreamCreate(&stream));

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
      unsigned int *points_num = nullptr;
      //unsigned int points_data_size = points_size * num_point_values * sizeof(float);
      checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
      checkCudaErrors(cudaMallocManaged((void **)&points_num, sizeof(unsigned int)));
      checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
      checkCudaErrors(cudaMemcpy(points_num, &points_size, sizeof(unsigned int), cudaMemcpyDefault));
      checkCudaErrors(cudaDeviceSynchronize());

      cudaEventRecord(start, stream);
      
      //pointpillar->doinfer(points_data, points_size, nms_pred);
      int temp=pointpillar->doinfer(points_data, points_size, nms_pred,nms_overlap_threshold,score_threshold);
      //auto pc_detection_arr = std::make_shared<visualization_msgs::msg::MarkerArray>();
      //std::vector<vision_msgs::msg::Detection3D> detections;
      visualization_msgs::msg::MarkerArray detections;
      visualization_msgs::msg::MarkerArray texts;
      for(int i=0; i<nms_pred.size(); i++) {
        //vision_msgs::msg::Detection3D detection;
        visualization_msgs::msg::Marker detection;
        visualization_msgs::msg::Marker text;
        text.lifetime=rclcpp::Duration::from_seconds(0.1);
        text.id=i;
        text.header.frame_id="/laser_link";
        text.ns = "text_shapes";
        text.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.4;
        text.color.b = 0;
        text.color.g = 0;
        text.color.r = 1;
        text.color.a = 0.3;
        text.pose.position.x = nms_pred[i].x;
        text.pose.position.y = nms_pred[i].y;
        text.pose.position.z = nms_pred[i].z+1;
        std::string text_message=std::to_string(nms_pred[i].id)+", "+std::to_string(nms_pred[i].score);
        text.text=text_message;
        texts.markers.emplace_back(text);
        
        //detection.results.resize(1); 
        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        vision_msgs::msg::BoundingBox3D bbox;
        geometry_msgs::msg::Pose center;
        geometry_msgs::msg::Vector3 size;
        geometry_msgs::msg::Point position;	
	 geometry_msgs::msg::Quaternion orientation;
        
        //detection.bbox.center.position.x = nms_pred[i].x;
        //detection.bbox.center.position.y = nms_pred[i].y;
        //detection.bbox.center.position.z = nms_pred[i].z;
        //detection.bbox.size.x = nms_pred[i].l;
        //detection.bbox.size.y = nms_pred[i].w;
        //detection.bbox.size.z = nms_pred[i].h;
        detection.id=i;
        detection.lifetime=rclcpp::Duration::from_seconds(0.1);
        detection.ns = "basic_shapes";
        detection.header.frame_id="/laser_link";
        detection.type=visualization_msgs::msg::Marker::CUBE;
        detection.action = visualization_msgs::msg::Marker::ADD;
        
        detection.pose.position.x = nms_pred[i].x;
        detection.pose.position.y = nms_pred[i].y;
        detection.pose.position.z = nms_pred[i].z;
        
        detection.scale.x = nms_pred[i].l;
        detection.scale.y = nms_pred[i].w;
        detection.scale.z = nms_pred[i].h;

        myQuaternion.setRPY(0, 0, nms_pred[i].rt);
        orientation = tf2::toMsg(myQuaternion);

        //detection.bbox.center.orientation = orientation;
        detection.pose.orientation.x = orientation.x;
        detection.pose.orientation.y = orientation.y;
        detection.pose.orientation.z = orientation.z;
        detection.pose.orientation.w = orientation.w;
        
        detection.color.r=1.0;
        detection.color.g=0.0;
        detection.color.b=0.0;
        detection.color.a=0.3;

        hyp.id = std::to_string(nms_pred[i].id);
        hyp.score = nms_pred[i].score;
        
        //detection.header = msg->header;
        
        //detection.results[0] = hyp;
        detections.markers.emplace_back(detection);
      }

      //pc_detection_arr->header = msg->header;
      //pc_detection_arr.detections = detections;
      //publisher_->publish(*pc_detection_arr);
      
      publisher_->publish(detections);
      publisher_text->publish(texts);
      //detections.markers.clear();
      //texts.markers.clear();
      
      cudaEventRecord(stop, stream);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);

      auto message = std_msgs::msg::String();
      message.data = "TIME: " + std::to_string(elapsedTime) + " ms, Objects detected: " + std::to_string(nms_pred.size());
      RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());

      checkCudaErrors(cudaFree(points_data));
      checkCudaErrors(cudaFree(points_num));
      nms_pred.clear();
      
  checkCudaErrors(cudaEventDestroy(start));
  checkCudaErrors(cudaEventDestroy(stop));
  checkCudaErrors(cudaStreamDestroy(stream));

  }
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{

  cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  //cvtColor(cv_ptr_img->image, cv_ptr_img->image, COLOR_BGR2RGB);
  Mat out_img;
  vector<_bbox> bbox;
  yolov5->objDetection(cv_ptr_img->image, out_img, bbox); //Box

  //bboxs_pub->publish(boxes);

  sensor_msgs::msg::Image img_msg;
  cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", out_img).toImageMsg(img_msg);
  img_msg.header.frame_id = "camera";
  obj_image_pub->publish(img_msg);
}

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_text;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr obj_image_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  //double start = ros::Time::now().toSec();
  
  //RCLCPP_INFO("score_threshold %f, nms_overlap_threshold %f", score_threshold_, nms_overlap_threshold_ );
  /*point_pillars_ptr_.reset(new PointPillars(score_threshold_, nms_overlap_threshold_, use_onnx_,
                                            pfe_onnx_file_, backbone_file_, pp_config_));*/
  //double end = ros::Time::now().toSec();

  //ROS_INFO("init time: %f", end - start);
 
 };
 
 
void Getinfo(void)
{
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem  >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
  printf("\n");
}

int loadData(const char *file, void **data, unsigned int *length)
{
  std::fstream dataFile(file, std::ifstream::in);

  if (!dataFile.is_open())
  {
	  std::cout << "Can't open files: "<< file<<std::endl;
	  return -1;
  }

  //get length of file:
  unsigned int len = 0;
  dataFile.seekg (0, dataFile.end);
  len = dataFile.tellg();
  dataFile.seekg (0, dataFile.beg);

  //allocate memory:
  char *buffer = new char[len];
  if(buffer==NULL) {
	  std::cout << "Can't malloc buffer."<<std::endl;
    dataFile.close();
	  exit(-1);
  }

  //read data as a block:
  dataFile.read(buffer, len);
  dataFile.close();

  *data = (void*)buffer;
  *length = len;
  return 0;  
}

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
  Getinfo();
  rclcpp::spin(std::make_shared<PointPillarsROS>());
  rclcpp::shutdown();
  return 0;
}
