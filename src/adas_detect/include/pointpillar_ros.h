/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
* @file point_pillars_ros.h
* @brief ROS interface for PointPillars
* @author Kosuke Murakami
* @date 2019/02/26
*/

#ifndef POINTS_PILLAR_ROS_H
#define POINTS_PILLAR_ROS_H

// headers in STL
#include <memory>
#include <vector>

// headers in ROS
//#include <ros/ros.h>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2_ros/transform_listener.h>
#include "vision_msgs/msg/detection3_d_array.hpp"

// headers in PCL
#include <pcl/io/pcd_io.h>

// headers in local files
//#include "pointpillar.h"

class PointPillarsROS : public rclcpp::Node
{
private:
  friend class TestClass;
  
  // initializer list

  bool has_subscribed_baselink_;
  const int NUM_POINT_FEATURE_;
  const int OUTPUT_NUM_BOX_FEATURE_;
  const float TRAINED_SENSOR_HEIGHT_;
  const float NORMALIZING_INTENSITY_VALUE_;
  const std::string BASELINK_FRAME_;
  // end initializer list

  // rosparam
  bool baselink_support_;
  bool use_tracking_;
  float score_threshold_;
  float nms_overlap_threshold_;
  bool use_onnx_;
  std::string pp_onnx_file_;
  std::string pp_config_;
  // end rosparam


  rclcpp::Node nh_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_objects_;

  float offset_z_from_trained_data_;

  //std::unique_ptr<PointPillars> point_pillars_ptr_;

  /**
  * @brief Get base_link to lidar transformation
  * @param[in] target_frameid Name of lidar frame_id to be targeted
  * @details Get transformation info
  */
  void getBaselinkToLidarTF(const std::string& target_frameid);

  /**
  * @brief Analyze tf information
  * @param[in] lidar2baselink transofomation info
  * @details Calculate z offset compared with trained sensor height and get rotation matrix
  */
  //void analyzeTFInfo(tf::StampedTransform lidar2baselink);

  /**
  * @brief Transform pose based on tf stamp info
  * @param[in] in_pose Target pose to be transformed
  * @param[in] tf TF stamp contains rotation matrix and translation matrix
  * @return geometry_msgs::Pose Transformed pose
  * @details Calculate transformed pose
  */
  //geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose, const tf::Transform& tf);

  /**
  * @brief callback for pointcloud
  * @param[in] input pointcloud from lidar sensor
  * @details Call point_pillars to get 3D bounding box
  */
  void pointsCallback(const sensor_msgs::msg::PointCloud2::ConstPtr input);

  /**
  * @brief convert pcl point to c++ array
  * @param[in] in_pcl_pc_ptr pointcloud in pcl format
  * @param[out] out_points_array converted pointcloud in c++ array format
  * @param[in] offset_z (default: 1.0) when using baselink_support, offset height based on current sensor configuration
  * @details convert pcl points to c++ array, plus offset z if it is necessary
  */
  void pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points_array,
                  const float offset_z = 0);

  void pclSave(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, int suffix);
  /**
  * @brief publish DetectedObject
  * @param[in] detections Network output bounding box
  * @param[in] in_header Header from pointcloud
  * @details Convert std::vector to DetectedObject, and publish them
  */
  void pubDetectedObject(
    const std::vector<float>& detections,
    const std::vector<int>& out_labels,
    const std::vector<float>& out_scores,
    const std_msgs::msg::Header& in_header);

public:
  PointPillarsROS();

  /**
  * @brief Create ROS pub/sub obejct
  * @details Create/Initializing ros pub/sub object
  */
  void createROSPubSub();
};

#endif  // POINTS_PILLAR_ROS_H
