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
#include <cstdlib>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <stdio.h>

#include <uuid/uuid.h>
#include <jsoncpp/json/json.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>

#include <custom_interfaces/msg/stampstring.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "yolov5.h"
#include "yoloparam.h"

#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <boost/filesystem.hpp>

using namespace std::chrono_literals;
using namespace std;
using namespace cv;
using namespace sensor_msgs;

class DsmYolov5 : public rclcpp::Node
{
public:
    yoloparam YP;
    void *YParam;
    DsmYolov5()
        : Node("dsm_yolov5")
    {

        this->declare_parameter<string>("dsm_image_sub_topic_name", "/dsm_video_frames");
        this->declare_parameter<string>("dsm_event_topic_name", "/dsm_json_pub");
        this->declare_parameter<string>("dsm_rtsp_url", "rtsp://admin:bdtd123456@192.168.1.42:554/h265/ch1/main/av_stream");

        this->declare_parameter<int>("device", 0);
        this->declare_parameter<double>("dsm_nms", 0.45);
        this->declare_parameter<double>("dsm_conf", 0.50);
        this->declare_parameter<int>("dsm_batch_size", 1);
        this->declare_parameter<int>("dsm_input_h", 640);
        this->declare_parameter<int>("dsm_input_w", 640);
        this->declare_parameter<int>("class_num", 11);
        this->declare_parameter<string>("dsm_engine_dir",
                                        "/home/nvidia/mdl_ws/src/dsm_cpp2/yolov5_tensorrt_build/best_end.engine");
        this->declare_parameter<float>("driver_ratio_", 0.3);
        this->declare_parameter<float>("head_ratio_", 0.3);
        this->declare_parameter<float>("eye_ratio_", 0.8);
        this->declare_parameter<float>("mouth_ratio_", 0.8);
        this->declare_parameter<float>("call_ratio_", 0.3);
        this->declare_parameter<float>("driver_leave_time_", 5.0);
        this->declare_parameter<float>("chaoyue_duration_time_", 3.0);

        // this->declare_parameter<float>("dsm_driver_leave_1", 7.0);
        // this->declare_parameter<float>("dsm_driver_leave_2", 5);
        // this->declare_parameter<float>("dsm_driver_leave_3", 3);
        this->declare_parameter<float>("ER_Time", 2.0);
        this->declare_parameter<float>("dsm_head_leave_1", 7.0);
        this->declare_parameter<float>("dsm_head_leave_2", 5.0);
        this->declare_parameter<float>("dsm_head_leave_3", 3.0);
        this->declare_parameter<float>("dsm_driver_drown_1", 7.0);
        this->declare_parameter<float>("dsm_driver_drown_2", 5.0);
        this->declare_parameter<float>("dsm_driver_drown_3", 3.0);
        this->declare_parameter<float>("dsm_driver_call_1", 7.0);
        this->declare_parameter<float>("dsm_driver_call_2", 5.0);
        this->declare_parameter<float>("dsm_driver_call_3", 3.0);
        this->declare_parameter<string>("car_number", "WLJ3BA");
        this->declare_parameter<bool>("dsm_yaml_button", true);

        std::string dsm_image_sub_topic_name = this->get_parameter("dsm_image_sub_topic_name").as_string();
        std::string dsm_event_topic_name = this->get_parameter("dsm_event_topic_name").as_string();
        rtsp_url = this->get_parameter("dsm_rtsp_url").as_string();
        this->get_parameter("device", YP.DEVICE);
        this->get_parameter("dsm_nms", YP.NMS_THRESH);
        this->get_parameter("dsm_conf", YP.CONF_THRESH);
        this->get_parameter("dsm_batch_size", YP.BATCH_SIZE);
        this->get_parameter("dsm_input_h", YP.INPUT_H);
        this->get_parameter("dsm_input_w", YP.INPUT_W);
        this->get_parameter("dsm_class_num", YP.CLASS_NUM);
        this->get_parameter("dsm_engine_dir", YP.ENGINE_DIR);
        this->get_parameter("driver_ratio_", driver_ratio_);
        this->get_parameter("head_ratio_", head_ratio_);
        this->get_parameter("eye_ratio_", eye_ratio_);
        this->get_parameter("mouth_ratio_", mouth_ratio_);
        this->get_parameter("call_ratio_", call_ratio_);
        this->get_parameter("driver_leave_time_", driver_leave_time_);
        this->get_parameter("chaoyue_duration_time_", chaoyue_duration_time_);
        this->get_parameter("ER_Time", ER_Time);
        this->get_parameter("dsm_head_leave_1", dsm_head_leave_1);
        this->get_parameter("dsm_head_leave_2", dsm_head_leave_2);
        this->get_parameter("dsm_head_leave_3", dsm_head_leave_3);
        this->get_parameter("dsm_driver_drown_1", dsm_driver_drown_1);
        this->get_parameter("dsm_driver_drown_2", dsm_driver_drown_2);
        this->get_parameter("dsm_driver_drown_3", dsm_driver_drown_3);
        this->get_parameter("dsm_driver_call_1", dsm_driver_call_1);
        this->get_parameter("dsm_driver_call_2", dsm_driver_call_2);
        this->get_parameter("dsm_driver_call_3", dsm_driver_call_3);
        this->get_parameter("car_number", car_number);
        this->get_parameter("dsm_yaml_button", dsm_yaml_button);

        YParam = &YP;
        yolov5 = new Yolov5(YParam);
        dsm_sub.subscribe(this, dsm_image_sub_topic_name);
        can_sub.subscribe(this, "/can_system_message");
        sync.reset(new Sync(MySyncPolicy(50), dsm_sub, can_sub));
        sync->registerCallback(std::bind(&DsmYolov5::callback, this, std::placeholders::_1, std::placeholders::_2));
        std::string dsm_drown_alert_pub_topic = "/htzgiot/" + car_number + "/thing/event/driver_warn_messages";
        std::string dsm_leave_alert_pub_topic = "/htzgiot/" + car_number + "/thing/event/driver_leave_messages";
        // subscription = this->create_subscription<sensor_msgs::msg::Image>(dsm_image_sub_topic_name, 10, std::bind(&DsmYolov5::callback, this, std::placeholders::_1));
        obj_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/dsm/drown_detection", 10);
        dsm_drown_alert_pub = this->create_publisher<std_msgs::msg::String>(dsm_drown_alert_pub_topic, 10);
        dsm_leave_alert_pub = this->create_publisher<std_msgs::msg::String>(dsm_leave_alert_pub_topic, 10);
        string_pub = this->create_publisher<std_msgs::msg::String>(dsm_event_topic_name, 10);
        brake_signal_pub = this->create_publisher<custom_interfaces::msg::Stampstring>("/dsm/can_message", 10);
    }

private:
    Yolov5 *yolov5;
    std::shared_ptr<spdlog::logger> logger;
    float driver_ratio_, head_ratio_, eye_ratio_, mouth_ratio_, call_ratio_, driver_leave_time_, chaoyue_duration_time_, dsm_head_leave_1, ER_Time,
        dsm_head_leave_2, dsm_head_leave_3, dsm_driver_drown_1, dsm_driver_drown_2, dsm_driver_drown_3, dsm_driver_call_1, dsm_driver_call_2, dsm_driver_call_3;
    std::string rtsp_url,car_number;
    bool log_revert = true;
    bool dsm_yaml_button;

    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    FILE *fp_shineng = NULL;
    FILE *fp_chaoyue = NULL;
    char shinengbuf[100] = {0};
    char chaoyuebuf[100] = {0};

    bool YOLOv5_Detection = true;

    bool strategy_button;
    bool chaoyue_duration;
    int shineng_button;
    int chaoyue_button;
    std::string shineng_button_str;
    std::string chaoyue_button_str;
    std::string can_message;
    int speed;
    int qianyin;
    int belt;
    int chair;
    int brake;
    int gear_d;
    int gear_r;
    int gear_n;
    int gear_p;
    int car_door;

    short driver_pid;
    short head_pid;
    short call_pid;
    short drown_pid;

    float driver_ratio = 1.0;
    float head_ratio = 1.0;
    float eye_ratio = 0.0;
    float mouth_ratio = 0.0;
    float call_ratio = 0.0;

    Json::Value driver;
    Json::Value head;
    Json::Value call;
    Json::Value drown;

    Json::Value driver_alert;
    Json::Value head_alert;
    Json::Value call_alert;
    Json::Value drown_alert;

    Json::Reader can_reader;
    Json::Value can_units;
    Json::Value can_pub;

    Json::FastWriter sw;

    std::string data_type;
    std::string event_id;
    // std::string image_sub_topic_name;
    const std::string ftp_image_path = "ftp://htzg:123456@192.168.1.102/dsm/images/";

    sensor_msgs::msg::Image img_msg;

    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> driver_start;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> call_start;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> head_start;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> drown_start;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> chaoyue_button_time;

    bool ER_driver = true;
    bool ER_head = true;
    bool ER_drown = true;
    bool ER_call = true;
    bool driver_first = true;
    bool call_first = true;
    bool head_first = true;
    bool drown_first = true;

    bool driver_second = true;
    bool call_second = true;
    bool head_second = true;
    bool drown_second = true;

    bool call_third = true;
    bool head_third = true;
    bool drown_third = true;

    bool call_fourth = true;
    bool head_fourth = true;
    bool drown_fourth = true;

    bool class_eye = false;
    bool class_mouth = false;

    const std::string images_filedir_prefix = "/home/nvidia/datalog/dsm/images/";
    const std::string videos_filedir_prefix = "/home/nvidia/datalog/dsm/videos/";

    typedef struct
    {
        std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> timestamp;
        std::string class_id;
    } _timeseries;
    std::vector<_timeseries> timeseries_vec;

    typedef struct
    {
        std::string id;
        std::string event_code;
        std::string begin_time;
        std::string end_time;
        std::string image_url;
    } _event_struct;
    _event_struct driver_event;
    _event_struct call_event;
    _event_struct head_event;
    _event_struct drown_event;

    /* typedef struct
    {
        std::string time;
        std::string carNumber;
        std::string carSpeed;
        std::string event;
    } _alert_struct;
    _alert_struct driver_alert;
    _alert_struct call_alert;
    _alert_struct head_alert;
    _alert_struct drown_alert;*/

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr obj_image_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dsm_drown_alert_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dsm_leave_alert_pub;
    rclcpp::Publisher<custom_interfaces::msg::Stampstring>::SharedPtr brake_signal_pub;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, custom_interfaces::msg::Stampstring> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync;
    message_filters::Subscriber<sensor_msgs::msg::Image> dsm_sub;
    message_filters::Subscriber<custom_interfaces::msg::Stampstring> can_sub;

    void callback(const sensor_msgs::msg::Image::ConstSharedPtr msg1, const custom_interfaces::msg::Stampstring::ConstSharedPtr msg2)
    {
        // spdlog
        can_pub["dsm"] = Json::Value("0");
        can_pub["0"] = Json::Value(0);
        can_pub["1"] = Json::Value(0);
        can_pub["2"] = Json::Value(0);
        //can_pub["3"] = Json::Value(0);
        can_pub["4"] = Json::Value(0);
        can_pub["5"] = Json::Value(0);
        //can_pub["6"] = Json::Value(1);
        can_message = msg2->data;
        can_reader.parse(can_message, can_units);
        speed = can_units["speed"].asInt();
        qianyin = can_units["qianyin"].asInt();
        gear_d = can_units["gear_d"].asInt();
        gear_r = can_units["gear_r"].asInt();
        gear_n = can_units["gear_n"].asInt();
        gear_p = can_units["gear_p"].asInt();
        brake = can_units["brake"].asInt();
        belt = can_units["belt"].asInt();
        chair = can_units["chair"].asInt();
        car_door = can_units["car_door"].asInt();
        // std::cerr << "car_door" << speed <<gear_d<<gear_r<<gear_n<<gear_p<<brake<<belt<<chair<<car_door<<qianyin<< std::endl;

        if (log_revert)
        {
            const std::string log_filedir_prefix = "/home/nvidia/datalog/dsm/system_logs/";
            time_t log_time = time(0);
            char log_time_tmp[32] = {NULL};
            strftime(log_time_tmp, sizeof(log_time_tmp), "%Y-%m-%d_%H-%M-%S", localtime(&log_time));
            const std::string log_filepathname = log_filedir_prefix + log_time_tmp + "/dsm_detect.log";
            logger = spdlog::basic_logger_mt("dsm_logger", log_filepathname);
            logger->set_pattern("[%Y-%m-%d_%H:%M:%S] [%n] [%^---%L---%$] %v");
            logger->info("received messages: {}", can_message.c_str());

            driver_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
            call_start = driver_start;
            head_start = driver_start;
            drown_start = driver_start;
            log_revert = false;
        }

        cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::TYPE_8UC3);
        Mat out_img;
        vector<_bbox> bbox;
        yolov5->objDetection(cv_ptr_img->image, out_img, bbox);
        if (YOLOv5_Detection)
        {
            RCLCPP_INFO(this->get_logger(), "dsm detection has been inintial");
            logger->info("dsm detection has been inintial");
            logger->flush();
            YOLOv5_Detection = false;
        }

        _timeseries dsm_timeseries_struct;

        // cur_time_tmp for make current dir(image, video)
        // end_time_tmp
        time_t cur_time = time(0);
        char cur_time_tmp[32] = {NULL};
        char end_time_tmp[32] = {NULL};
        strftime(cur_time_tmp, sizeof(cur_time_tmp), "%Y-%m-%d", localtime(&cur_time));
        strftime(end_time_tmp, sizeof(end_time_tmp), "%Y-%m-%d_%H-%M-%S", localtime(&cur_time));

        std::string image_file_path = images_filedir_prefix + cur_time_tmp;
        std::string video_file_path = videos_filedir_prefix + cur_time_tmp;

        // mkdir
        // struct stat st;
        // if (stat(image_file_path.c_str(), &st) == -1)
        // {
        //   mkdir(image_file_path.c_str(), 0777);
        // }
        // if (stat(video_file_path.c_str(), &st) == -1)
        // {
        //   mkdir(video_file_path.c_str(), 0777);
        // }

        boost::filesystem::path image_path(images_filedir_prefix + cur_time_tmp);
        boost::filesystem::path video_path(videos_filedir_prefix + cur_time_tmp);
        if (!boost::filesystem::exists(image_path))
        {
            boost::filesystem::create_directories(image_path);
        }
        if (!boost::filesystem::exists(video_path))
        {
            boost::filesystem::create_directories(video_path);
        }

        auto current_time = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        dsm_timeseries_struct.timestamp = current_time;

        // shinengbuf[100] = {0};
        // chaoyuebuf[100] = {0};
        fp_shineng = popen("cat /sys/class/gpio/PA.02/value", "r");
        fp_chaoyue = popen("cat /sys/class/gpio/PG.07/value", "r");
        if (fp_shineng)
        {
            int ret = fread(shinengbuf, 1, sizeof(shinengbuf) - 1, fp_shineng);
            if (ret > 0)
            {
                shineng_button_str = shinengbuf;
            }
            pclose(fp_shineng);
        }
        if (fp_chaoyue)
        {
            int ret = fread(chaoyuebuf, 1, sizeof(chaoyuebuf) - 1, fp_chaoyue);
            if (ret > 0)
            {
                chaoyue_button_str = chaoyuebuf;
            }
            pclose(fp_chaoyue);
        }
        shineng_button = std::stoi(shineng_button_str);
        chaoyue_button = std::stoi(chaoyue_button_str);
        if (shineng_button)
        {
            strategy_button = false;
            logger->info("[{} {}] dsm has been shutdown", shineng_button, chaoyue_button);
            RCLCPP_INFO(this->get_logger(), "dsm has been shutdown");
        }
        else
        {
            strategy_button = true;
            if (!chaoyue_button)
            {
                chaoyue_button_time = current_time;
                chaoyue_duration = true;
            }
            if (chaoyue_duration)
            {
                if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - chaoyue_button_time).count()) < (chaoyue_duration_time_ * 1000))
                {
                    strategy_button = false;
                }
                else
                {
                    chaoyue_duration = false;
                }
            }
        }
        class_eye = false;
        class_mouth = false;
        for (size_t i = 0; i < bbox.size(); i++)
        {
            dsm_timeseries_struct.class_id += bbox[i].class_id;
            if (bbox[i].class_id == "0" || bbox[i].class_id == "2")
            {
                class_eye = true;
            }
            if (bbox[i].class_id == "1" || bbox[i].class_id == "3")
            {
                class_mouth = true;
            }
        }
        if (timeseries_vec.empty() || int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - timeseries_vec[0].timestamp).count()) < 3000)
        {
            timeseries_vec.push_back(dsm_timeseries_struct);
        }
        else
        {
            while (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - timeseries_vec[0].timestamp).count()) > 3000)
            {
                if (timeseries_vec.size() < 2)
                {
                    break;
                }
                else
                {
                    timeseries_vec.erase(timeseries_vec.begin());
                }
            }
            timeseries_vec.push_back(dsm_timeseries_struct);

            std::string one_second_seriesid;
            float one_sec_timeseries_len = timeseries_vec.size() / 3 + 1.0;
            std::string two_second_seriesid;
            float two_sec_timeseries_len = timeseries_vec.size() * 2 / 3 + 1.0;
            std::string three_second_seriesid;
            float three_sec_timeseries_len = timeseries_vec.size();
            int count_one_second_i = 0;

            for (auto timeseries_vec_item : timeseries_vec)
            {
                auto duration_time = timeseries_vec[timeseries_vec.size() - 1].timestamp - timeseries_vec_item.timestamp;
                if (duration_time.count() < 2000)
                {
                    two_second_seriesid += timeseries_vec_item.class_id;
                }
                if (duration_time.count() < 1000)
                {
                    one_second_seriesid += timeseries_vec_item.class_id;
                    count_one_second_i++;
                }
                three_second_seriesid += timeseries_vec_item.class_id;
            }

            float one_sec_driver_num = count(one_second_seriesid.begin(), one_second_seriesid.end(), '5'); // int ?
            float one_sec_head_num = count(one_second_seriesid.begin(), one_second_seriesid.end(), '6');
            //float two_sec_eye_num = count(two_second_seriesid.begin(), two_second_seriesid.end(), '0');
            float three_sec_eye_num = count(three_second_seriesid.begin(), three_second_seriesid.end(), '0');
            float three_sec_mouth_num = count(three_second_seriesid.begin(), three_second_seriesid.end(), '3');
            float three_sec_call_num = count(three_second_seriesid.begin(), three_second_seriesid.end(), '4');

            driver_ratio = one_sec_driver_num / one_sec_timeseries_len;
            head_ratio = one_sec_head_num / one_sec_timeseries_len;
            eye_ratio = three_sec_eye_num / two_sec_timeseries_len;
            mouth_ratio = three_sec_mouth_num / three_sec_timeseries_len;
            call_ratio = three_sec_call_num / three_sec_timeseries_len;
            std::cerr << "timeseries_vec.size()  " << timeseries_vec.size() << std::endl;

            if (strategy_button && dsm_yaml_button)
            {
                std::cerr << "speed: " << speed << "  gear_n:  " << gear_n << " gear_p:  " << gear_p << std::endl;
                if ((driver_ratio < driver_ratio_) && (class_eye == false) && (class_mouth == false))
                {
                    if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - driver_start).count()) < (ER_Time * 1000))
                    {
                        driver_first = false;
                    }
                    std::cerr << "driver_ratio  " << driver_ratio << "driver_ratio_:  " << driver_ratio_ << std::endl;
                    if (speed || (!gear_n) || (!gear_p))
                    {
                        if (driver_first == true || ER_driver)
                        {
                            ER_driver = false;
                            driver_first = false;
                            driver_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
                            time_t current_time_ = time(0);
                            char current_date[32] = {NULL};
                            strftime(current_date, sizeof(current_date), "%Y-%m-%d_%H-%M-%S", localtime(&current_time_));
                            auto driver_uid_time = std::chrono::system_clock::now().time_since_epoch().count();
                            auto driver_uid = std::rand() + driver_uid_time;
                            driver_event.id = std::to_string(driver_uid);
                            std::string driver_image_fname = image_file_path + "/driver" + current_date + driver_event.id + ".jpg";

                            cv::imwrite(driver_image_fname, out_img);
                            driver_event.event_code = "300004";
                            driver_event.begin_time = current_date;
                            driver_event.image_url = ftp_image_path + cur_time_tmp + "/driver" + current_date + driver_event.id + ".jpg";

                            pid_t driver_video_pid = fork();
                            driver_pid = driver_video_pid;
                            if (driver_video_pid == 0)
                            {
                                setsid();
                                std::string driver_video_fname = video_file_path + "/driver" + current_date + cur_time_tmp + driver_event.id + ".mp4";
                                const char *driver_video_cfname = driver_video_fname.c_str();
                                char *driver_video_cfnamec = (char *)driver_video_cfname;
                                const char *rtsp_url_ = rtsp_url.c_str();
                                char *rtsp_url_c = (char *)rtsp_url_;
                                char *const ffmpeg_record_argv[] =
                                    {"ffmpeg", "-i", rtsp_url_c, "-t", "10", "-c", "copy", driver_video_cfnamec, nullptr};
                                execvp("ffmpeg", ffmpeg_record_argv);
                                _exit(0);
                            }
                            driver["id"] = Json::Value(driver_event.id);
                            driver["event_code"] = Json::Value(driver_event.event_code);
                            driver["begin_time"] = Json::Value(driver_event.begin_time);
                            driver["image_url"] = Json::Value(driver_event.image_url);

                            driver_alert["time"] = Json::Value(current_date);
                            driver_alert["carNumber"] = Json::Value(car_number);
                            driver_alert["carSpeed"] = Json::Value(speed);
                            driver_alert["event"] = Json::Value(driver_event.event_code);
                        }
                        if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - driver_start).count()) > (driver_leave_time_ * 1000))
                        {
                            if (driver_second)
                            {
                                logger->info("[{} {}] driver leave alarm: driver dont exist start to braking", shineng_button, chaoyue_button);
                                driver_event.event_code = "300001";
                                driver["event_code"] = Json::Value(driver_event.event_code);
                                driver_alert["event"] = Json::Value(driver_event.event_code);
                                driver_second = false;
                            }
                            //auto brake_msg = std_msgs::msg::String();
                            if (speed)
                            {
                                can_pub["0"] = Json::Value(100);
                                can_pub["1"] = Json::Value(2);
                            }
                            else
                            {
                                can_pub["2"] = Json::Value(1);
                            }
                        }
                    }
                }
                else
                {
                    if (driver_first == false)
                    {
                        // kill(-driver_pid, SIGTERM);
                        driver_event.end_time = end_time_tmp;
                        driver["end_time"] = Json::Value(driver_event.end_time);
                        auto string_msg = std_msgs::msg::String();
                        string_msg.data = sw.write(driver);

                        

                        auto alert_string_msg = std_msgs::msg::String();
                        alert_string_msg.data = sw.write(driver_alert);
                        logger->info("[{} {}] driver leave_alert alarm: {}", shineng_button, chaoyue_button, alert_string_msg.data);
                        //logger->flush();
                        
                        logger->info("[{} {}] driver leave alarm: {}", shineng_button, chaoyue_button, driver.isMember("begin_time"));
                        logger->flush();
                        if (driver.isMember("begin_time")){
                        logger->info("[{} {}] driver leave alarm: {}", shineng_button, chaoyue_button, string_msg.data);
                        logger->info("[{} {}] driver leave_alert alarm: {}", shineng_button, chaoyue_button, alert_string_msg.data);
                        logger->flush();
                        driver.clear();
                        driver_alert.clear();
                        string_pub->publish(string_msg);
                        dsm_leave_alert_pub->publish(alert_string_msg);}
                    }
                    driver_first = true;
                    driver_second = true;
                    if ((head_ratio < head_ratio_) && (class_eye == false) && (class_mouth == false))
                    {
                        if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - head_start).count()) < (ER_Time * 1000))
                        {
                            head_first = false;
                        }
                        std::cerr << "head_ratio  " << head_ratio << "head_ratio_:  " << head_ratio_ << std::endl;
                        if (head_first == true || ER_head)
                        {
                            ER_head = false;
                            head_first = false;
                            head_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
                            time_t current_time_ = time(0);
                            char current_date[32] = {NULL};
                            strftime(current_date, sizeof(current_date), "%Y-%m-%d_%H-%M-%S", localtime(&current_time_));
                            auto head_uid_time = std::chrono::system_clock::now().time_since_epoch().count();
                            auto head_uid = std::rand() + head_uid_time;
                            head_event.id = std::to_string(head_uid);
                            std::string head_image_fname = image_file_path + "/head" + current_date + head_event.id + ".jpg";

                            cv::imwrite(head_image_fname, out_img);
                            head_event.event_code = "200024";
                            head_event.begin_time = current_date;
                            head_event.image_url = ftp_image_path + cur_time_tmp + "/head" + current_date + head_event.id + ".jpg";

                            pid_t head_video_pid = fork();
                            head_pid = head_video_pid;
                            if (head_video_pid == 0)
                            {
                                setsid();
                                std::string head_video_fname = video_file_path + "/head" + cur_time_tmp + current_date + head_event.id + ".mp4";
                                const char *head_video_cfname = head_video_fname.c_str();
                                char *head_video_cfnamec = (char *)head_video_cfname;
                                const char *rtsp_url_ = rtsp_url.c_str();
                                char *rtsp_url_c = (char *)rtsp_url_;
                                char *const ffmpeg_record_argv[] =
                                    {"ffmpeg", "-i", rtsp_url_c, "-t", "10", "-c", "copy", head_video_cfnamec, nullptr};
                                execvp("ffmpeg", ffmpeg_record_argv);
                                _exit(0);
                            }
                            head["id"] = Json::Value(head_event.id);
                            head["event_code"] = Json::Value(head_event.event_code);
                            head["begin_time"] = Json::Value(head_event.begin_time);
                            head["image_url"] = Json::Value(head_event.image_url);

                            head_alert["time"] = Json::Value(current_date);
                            head_alert["carNumber"] = Json::Value(car_number);
                            head_alert["carSpeed"] = Json::Value(speed);
                            head_alert["event"] = Json::Value(head_event.event_code);
                        }
                        //auto brake_msg = std_msgs::msg::String();

                        std::cerr << "head_duration:  " << int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - head_start).count()) << std::endl;
                        if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - head_start).count()) > (dsm_head_leave_1 * 1000))
                        {
                            if (head_fourth)
                            {
                                head_event.event_code = "200021";
                                head["event_code"] = Json::Value(head_event.event_code);
                                head_alert["event"] = Json::Value(head_event.event_code);
                                head_fourth = false;
                            }
                        can_pub["0"] = Json::Value(70);
                        can_pub["1"] = Json::Value(3);
                        }
                        else
                        {
                            if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - head_start).count()) > (dsm_head_leave_2 * 1000))
                            {
                                if (head_third)
                                {
                                    head_event.event_code = "200022";
                                    head["event_code"] = Json::Value(head_event.event_code);
                                    head_alert["event"] = Json::Value(head_event.event_code);
                                    head_third = false;
                                }
                        can_pub["0"] = Json::Value(50);
                        can_pub["1"] = Json::Value(2);                                
                            }
                            else
                            {
                                if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - head_start).count()) > (dsm_head_leave_3 * 1000))
                                {
                                    if (head_second)
                                    {
                                        head_event.event_code = "200023";
                                        head["event_code"] = Json::Value(head_event.event_code);
                                        head_alert["event"] = Json::Value(head_event.event_code);
                                        head_second = false;
                                    }
                        can_pub["0"] = Json::Value(20);
                        can_pub["1"] = Json::Value(1);
                                }
                            }
                        }
                    }
                    else
                    {
                        if (head_first == false)
                        {
                            // kill(-head_pid, SIGTERM);
                            head_event.end_time = end_time_tmp;
                            head["end_time"] = Json::Value(head_event.end_time);
                            auto head_string_msg = std_msgs::msg::String();
                            head_string_msg.data = sw.write(head);
                            
                            

                            auto alert_string_msg = std_msgs::msg::String();
                            alert_string_msg.data = sw.write(head_alert);
                            
                            //logger->flush();
                            
                            if (head.isMember("begin_time")){
                            logger->info("[{} {}] driver head leave alarm: {}", shineng_button, chaoyue_button, head_string_msg.data);
                            logger->info("[{} {}] driver head_alert alarm: {}", shineng_button, chaoyue_button, alert_string_msg.data);
                            logger->flush(); 
                            head.clear();    
                            head_alert.clear();                       
                            string_pub->publish(head_string_msg);
                            dsm_drown_alert_pub->publish(alert_string_msg);}
                        }
                        head_first = true;
                        head_second = true;
                        head_third = true;
                        head_fourth = true;
                    }
                    if (eye_ratio > eye_ratio_ || mouth_ratio > mouth_ratio_)
                    {
                        int ER_drown_duration = int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - drown_start).count());
                        std::cerr << "ER_drown_duration  " << ER_drown_duration << "ER_Time:  " << ER_Time * 1000 << std::endl;
                        std::cerr << "eye_ratio  " << eye_ratio << "eye_ratio_:  " << eye_ratio_ << std::endl;
                        std::cerr << "mouth_ratio  " << mouth_ratio << "mouth_ratio_:  " << mouth_ratio_ << std::endl;

                        if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - drown_start).count()) < (ER_Time * 1000))
                        {
                            drown_first = false;
                        }

                        if (drown_first == true || ER_drown)
                        {
                            ER_drown = false;
                            drown_first = false;
                            drown_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
                            time_t current_time_ = time(0);
                            char current_date[32] = {NULL};
                            strftime(current_date, sizeof(current_date), "%Y-%m-%d_%H-%M-%S", localtime(&current_time_));
                            auto drown_uid_time = std::chrono::system_clock::now().time_since_epoch().count();
                            auto drown_uid = std::rand() + drown_uid_time;
                            drown_event.id = std::to_string(drown_uid);
                            std::string drown_image_fname = image_file_path + "/sleep" + current_date + drown_event.id + ".jpg";
                            cv::imwrite(drown_image_fname, out_img);
                            drown_event.event_code = "200004";
                            drown_event.begin_time = current_date;
                            drown_event.image_url = ftp_image_path + cur_time_tmp + "/sleep" + current_date + drown_event.id + ".jpg";

                            pid_t drown_video_pid = fork();
                            drown_pid = drown_video_pid;
                            if (drown_video_pid == 0)
                            {
                                setsid();
                                std::string drown_video_fname = video_file_path + "/sleep" + current_date + cur_time_tmp + drown_event.id + ".mp4";
                                const char *drown_video_cfname = drown_video_fname.c_str();
                                char *drown_video_cfnamec = (char *)drown_video_cfname;
                                const char *rtsp_url_ = rtsp_url.c_str();
                                char *rtsp_url_c = (char *)rtsp_url_;
                                char *const ffmpeg_record_argv[] =
                                    {"ffmpeg", "-i", rtsp_url_c, "-t", "10", "-c", "copy", drown_video_cfnamec, nullptr};
                                execvp("ffmpeg", ffmpeg_record_argv);
                                _exit(0);
                            }
                            drown["id"] = Json::Value(drown_event.id);
                            drown["event_code"] = Json::Value(drown_event.event_code);
                            drown["begin_time"] = Json::Value(drown_event.begin_time);
                            drown["image_url"] = Json::Value(drown_event.image_url);

                            drown_alert["time"] = Json::Value(current_date);
                            drown_alert["carNumber"] = Json::Value(car_number);
                            drown_alert["carSpeed"] = Json::Value(speed);
                            drown_alert["event"] = Json::Value(drown_event.event_code);
                        }
                        std::cerr << "drown_duration:  " << int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - drown_start).count()) << std::endl;
                        if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - drown_start).count()) > (dsm_driver_drown_1 * 1000))
                        {
                            std::cerr << "drown_fourth:  " << drown_fourth << std::endl;
                            if (drown_fourth)
                            {
                                drown_event.event_code = "200001";
                                drown["event_code"] = Json::Value(drown_event.event_code);
                                drown_alert["event"] = Json::Value(drown_event.event_code);
                                drown_fourth = false;
                            }
                        can_pub["0"] = Json::Value(70);
                        can_pub["1"] = Json::Value(3);
                        }
                        else
                        {
                            if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - drown_start).count()) > (dsm_driver_drown_2 * 1000))
                            {
                                std::cerr << "drown_thirds:  " << drown_third << std::endl;
                                if (drown_third)
                                {
                                    drown_event.event_code = "200002";
                                    drown["event_code"] = Json::Value(drown_event.event_code);
                                    drown_alert["event"] = Json::Value(drown_event.event_code);
                                    drown_third = false;
                                }
                        can_pub["0"] = Json::Value(50);
                        can_pub["1"] = Json::Value(2);
                            }
                            else
                            {
                                if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - drown_start).count()) > (dsm_driver_drown_3 * 1000))
                                {
                                    std::cerr << "drown_second:  " << drown_second << std::endl;
                                    if (drown_second)
                                    {
                                        drown_event.event_code = "200003";
                                        drown["event_code"] = Json::Value(drown_event.event_code);
                                        drown_alert["event"] = Json::Value(drown_event.event_code);
                                        drown_second = false;
                                    }
                        can_pub["0"] = Json::Value(20);
                        can_pub["1"] = Json::Value(1);
                                }
                            }
                        }
                    }
                    else
                    {
                        if (drown_first == false)
                        {
                            // kill(-drown_pid, SIGTERM);
                            drown_event.end_time = end_time_tmp;
                            drown["end_time"] = Json::Value(drown_event.end_time);
                            auto drown_string_msg = std_msgs::msg::String();
                            drown_string_msg.data = sw.write(drown);

                            
                            

                            auto alert_string_msg = std_msgs::msg::String();
                            alert_string_msg.data = sw.write(drown_alert);
                            
                            //logger->flush();
                            
                            if (drown.isMember("begin_time")){
                            logger->info("[{} {}] driver sleep alarm: {}", shineng_button, chaoyue_button, drown_string_msg.data);
                            logger->info("[{} {}] driver sleep_alert alarm: {}", shineng_button, chaoyue_button, alert_string_msg.data);
                            logger->flush(); 
                            drown.clear();   
                            drown_alert.clear();                        
                            dsm_drown_alert_pub->publish(alert_string_msg);
                            string_pub->publish(drown_string_msg);}
                        }
                        drown_first = true;
                        drown_second = true;
                        drown_third = true;
                        drown_fourth = true;
                    }
                    if (call_ratio > call_ratio_)
                    {
                        std::cerr << "call_ratio  " << call_ratio << "call_ratio_:  " << call_ratio_ << std::endl;
                        if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - call_start).count()) < (ER_Time * 1000))
                        {
                            call_first = false;
                        }
                        if (call_first == true || ER_call)
                        {
                            ER_call = false;
                            call_first = false;
                            call_start = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
                            time_t current_time_ = time(0);
                            char current_date[32] = {NULL};
                            strftime(current_date, sizeof(current_date), "%Y-%m-%d_%H-%M-%S", localtime(&current_time_));
                            auto call_uid_time = std::chrono::system_clock::now().time_since_epoch().count();
                            auto call_uid = std::rand() + call_uid_time;
                            call_event.id = std::to_string(call_uid);
                            std::string call_image_fname = image_file_path + "/call" + current_date + call_event.id + ".jpg";
                            cv::imwrite(call_image_fname, out_img);
                            call_event.event_code = "200014";
                            call_event.begin_time = current_date;
                            call_event.image_url = ftp_image_path + cur_time_tmp + "/call" + current_date + call_event.id + ".jpg";
                            pid_t call_video_pid = fork();
                            call_pid = call_video_pid;
                            if (call_video_pid == 0)
                            {
                                setsid();
                                std::string call_video_fname = video_file_path + "/call" + cur_time_tmp + current_date + call_event.id + ".mp4";
                                const char *call_video_cfname = call_video_fname.c_str();
                                char *call_video_cfnamec = (char *)call_video_cfname;
                                const char *rtsp_url_ = rtsp_url.c_str();
                                char *rtsp_url_c = (char *)rtsp_url_;
                                char *const ffmpeg_record_argv[] =
                                    {"ffmpeg", "-i", rtsp_url_c, "-t", "10", "-c", "copy", call_video_cfnamec, nullptr};
                                execvp("ffmpeg", ffmpeg_record_argv);
                                _exit(0);
                            }
                            call["id"] = Json::Value(call_event.id);
                            call["event_code"] = Json::Value(call_event.event_code);
                            call["begin_time"] = Json::Value(call_event.begin_time);
                            call["image_url"] = Json::Value(call_event.image_url);

                            call_alert["time"] = Json::Value(current_date);
                            call_alert["carNumber"] = Json::Value(car_number);
                            call_alert["carSpeed"] = Json::Value(speed);
                            call_alert["event"] = Json::Value(call_event.event_code);
                        }
                        std::cerr << "call_duration:  " << int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - call_start).count()) << std::endl;
                        if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - call_start).count()) > (dsm_driver_call_1 * 1000))
                        {
                            std::cerr << "call_fourth:  " << call_fourth << std::endl;
                            if (call_fourth)
                            {
                                call_event.event_code = "200011";
                                call["event_code"] = Json::Value(call_event.event_code);
                                call_alert["event"] = Json::Value(call_event.event_code);
                                call_fourth = false;
                            }
                        can_pub["0"] = Json::Value(70);
                        can_pub["1"] = Json::Value(3);                            
                        }
                        else
                        {
                            if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - call_start).count()) > (dsm_driver_call_2 * 1000))
                            {
                                std::cerr << "call_third:  " << call_third << std::endl;
                                if (call_third)
                                {
                                    call_event.event_code = "200012";
                                    call["event_code"] = Json::Value(call_event.event_code);
                                    call_alert["event"] = Json::Value(call_event.event_code);
                                    call_third = false;
                                }
                        can_pub["0"] = Json::Value(50);
                        can_pub["1"] = Json::Value(2);                                
                            }
                            else
                            {
                                if (int(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - call_start).count()) > (dsm_driver_call_3 * 1000))
                                {
                                    std::cerr << "call_second:  " << call_second << std::endl;
                                    if (call_second)
                                    {

                                        call_event.event_code = "200013";
                                        call["event_code"] = Json::Value(call_event.event_code);
                                        call_alert["event"] = Json::Value(call_event.event_code);
                                        call_second = false;
                                    }
                        can_pub["0"] = Json::Value(20);
                        can_pub["1"] = Json::Value(1);                                    
                                }
                            }
                        }
                    }
                    else
                    {
                        if (call_first == false)
                        {
                            // kill(-call_pid, SIGTERM);
                            call_event.end_time = end_time_tmp;
                            call["end_time"] = Json::Value(call_event.end_time);
                            auto call_string_msg = std_msgs::msg::String();
                            call_string_msg.data = sw.write(call);

                            
                            

                            auto alert_string_msg = std_msgs::msg::String();
                            alert_string_msg.data = sw.write(call_alert);
                            
                            //logger->flush();
                            
                            if (call.isMember("begin_time")){
                            logger->info("[{} {}] driver call alarm: {}", shineng_button, chaoyue_button, call_string_msg.data);
                            logger->info("[{} {}] driver call_alert alarm: {}", shineng_button, chaoyue_button, alert_string_msg.data);
                            logger->flush(); 
                            call.clear(); 
                            call_alert.clear();                          
                            dsm_drown_alert_pub->publish(alert_string_msg);
                            string_pub->publish(call_string_msg);}
                        }
                        call_first = true;
                        call_second = true;
                        call_third = true;
                        call_fourth = true;
                    }
                }

                sensor_msgs::msg::Image img_msg;
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", out_img).toImageMsg(img_msg);
                img_msg.header.frame_id = "camera";
                // auto string_msg = std_msgs::msg::String();
                obj_image_pub->publish(img_msg);
            }
        }
                        auto can_pub_msg = custom_interfaces::msg::Stampstring();
                auto stamp = this->get_clock()->now();
                can_pub_msg.header.stamp = stamp;
                can_pub_msg.data = sw.write(can_pub);
                brake_signal_pub->publish(can_pub_msg);
                can_pub.clear();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DsmYolov5>());
    rclcpp::shutdown();
    return 0;
}
