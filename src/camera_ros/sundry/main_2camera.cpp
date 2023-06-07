#include <iostream>
#include <thread>
#include <mutex>
#include <time.h>
#include "spdlog/spdlog.h"
#include <stdlib.h>
#include "spdlog/sinks/basic_file_sink.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;

mutex dsm_frame_mutex;
mutex adas_frame_mutex;
Mat dsm_frame;
Mat adas_frame;
Size dsize = Size(640, 480);

Rect dsm_rect(50, 50, 640, 480);

std::string dsm_rtsp_url = "rtsp://admin:bdtd123456@192.168.1.39:554/h265/ch1/main/av_stream";
std::string adas_rtsp_url = "rtsp://admin:hf123456@192.168.1.64:554/h265/ch1/main/av_stream";
std::shared_ptr<spdlog::logger> logger;
void read_Dsm_RTSP_Video()
{
    VideoCapture cap(dsm_rtsp_url);
    if (!cap.isOpened())
    {
        throw std::runtime_error("Failed to open RTSP stream");
        exit(1);
    }
    logger->info("Camera start successfully");
    logger->flush();

    while (rclcpp::ok())
    {
        Mat temp, temp_size;
        cap >> temp;
        // dsm_temp_size = dsm_temp(dsm_rect);
        resize(temp, temp_size, dsize, 0, 0, INTER_AREA);
        dsm_frame_mutex.lock();
        dsm_frame = temp_size.clone();
        dsm_frame_mutex.unlock();
    }
}

void read_Adas_RTSP_Video()
{
    VideoCapture cap(adas_rtsp_url);
    if (!cap.isOpened())
    {
        throw std::runtime_error("Failed to open RTSP stream");
        exit(1);
    }
    logger->info("Camera start successfully");
    logger->flush();

    while (rclcpp::ok())
    {
        Mat temp, temp_size;
        cap >> temp;
        // dsm_temp_size = dsm_temp(dsm_rect);
        resize(temp, temp_size, dsize, 0, 0, INTER_AREA);
        adas_frame_mutex.lock();
        adas_frame = temp_size.clone();
        adas_frame_mutex.unlock();
    }
}

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        this->declare_parameter<string>("dsm_rtsp_url", "rtsp://admin:bdtd123456@192.168.1.39:554/h265/ch1/main/av_stream");
        dsm_rtsp_url = this->get_parameter("dsm_rtsp_url").as_string();
        dsm_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/dsm_video_frames", 10);
        adas_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/adas_video_frames", 10);
        const std::string log_filedir_prefix = "/home/nvidia/datalog/ftp/dsm/system_log/";
        time_t log_time = time(0);
        char log_time_tmp[32] = {NULL};
        strftime(log_time_tmp, sizeof(log_time_tmp), "%Y-%m-%d_%H-%M-%S", localtime(&log_time));
        const std::string log_filepathname = log_filedir_prefix + log_time_tmp + "/camera_ros.log";
        logger = spdlog::basic_logger_mt("basic_logger", log_filepathname);

        thread t1(read_Dsm_RTSP_Video);
        thread t2(read_Adas_RTSP_Video);
        logger->info("Dsm  detection has been inintial");
        logger->info("Adas  detection has been inintial");
        logger->flush();
        while (rclcpp::ok())
        {
            Mat dsm_current_frame, adas_current_frame;

            dsm_frame_mutex.lock();
            dsm_current_frame = dsm_frame.clone();
            dsm_frame_mutex.unlock();

            adas_frame_mutex.lock();
            adas_current_frame = adas_frame.clone();
            adas_frame_mutex.unlock();

            if (!dsm_current_frame.empty())
            {
                sensor_msgs::msg::Image dsm_current_msg;
                // cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dsm_current_frame).toImageMsg(current_msg);
                cv_bridge::CvImage(std_msgs::msg::Header(), "8UC3", dsm_current_frame).toImageMsg(dsm_current_msg);
                double time_s = this->get_clock()->now().seconds();
                double time_ns = this->get_clock()->now().nanoseconds();
                dsm_current_msg.header.stamp.sec = time_s;
                dsm_current_msg.header.stamp.nanosec = time_ns;
                dsm_image_pub->publish(dsm_current_msg);
            }

            if (!adas_current_frame.empty())
            {
                sensor_msgs::msg::Image adas_current_msg;
                // cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dsm_current_frame).toImageMsg(current_msg);
                cv_bridge::CvImage(std_msgs::msg::Header(), "8UC3", adas_current_frame).toImageMsg(adas_current_msg);
                double time_s = this->get_clock()->now().seconds();
                double time_ns = this->get_clock()->now().nanoseconds();
                adas_current_msg.header.stamp.sec = time_s;
                adas_current_msg.header.stamp.nanosec = time_ns;
                adas_image_pub->publish(adas_current_msg);
            }
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dsm_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr adas_image_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}