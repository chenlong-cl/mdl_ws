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
#include "sensor_msgs/msg/compressed_image.hpp"

#include <chrono>

using namespace std;
using namespace cv;

mutex frame_mutex;

Mat frame, current_frame;

std::shared_ptr<spdlog::logger> logger;
void readRTSPVideo(const Size &res_image_size, const Rect &res_image_rect, const std::string & url_)
{
    VideoCapture cap(url_);
    if (!cap.isOpened())
    {
        throw std::runtime_error("Failed to open RTSP stream");
        logger->info("Failed to open RTSP stream");
        logger->flush();
        exit(1);
    }
    // cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    int width = cap.get(CAP_PROP_FRAME_WIDTH);
    int fps = cap.get(CAP_PROP_FPS);
    int buf_size = cap.get(CAP_PROP_BUFFERSIZE);

    logger->info("Camera start successfully");
    logger->info("Camera_property: height: {0}, width: {1}, fps: {2}, buf_size: {3}", height, width, fps, buf_size);
    logger->flush();

    Mat temp, rect_temp, temp_size;

    while (rclcpp::ok())
    {

        cap >> temp;

        rect_temp = temp(res_image_rect);

        resize(rect_temp, temp_size, res_image_size, 0, 0, INTER_AREA);
        frame_mutex.lock();
        frame = temp_size;
        frame_mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        this->declare_parameter<string>("url", "rtsp://admin:bdtd123456@192.168.1.39:554/h265/ch1/main/av_stream");
        this->declare_parameter<int>("output_h", 640);
        this->declare_parameter<int>("output_w", 640);
        this->declare_parameter<string>("image_pub_topic_name", "/video_frames");
        this->declare_parameter<string>("compressed_image_pub_topic_name", "compressed/video_frames");
        this->declare_parameter<int>("frame_rate", 20);
        this->declare_parameter<string>("log_filedir_prefix", "~/Projects/mdl_ws/datalog/mdl_ros_logs/");
        this->declare_parameter<int>("rect_x", 0);
        this->declare_parameter<int>("rect_y", 0);
        this->declare_parameter<int>("rect_h", 1080);
        this->declare_parameter<int>("rect_w", 1920);

        std::string url = this->get_parameter("url").as_string();
        std::string image_pub_topic_name = this->get_parameter("image_pub_topic_name").as_string();
        std::string compressed_image_pub_topic_name = this->get_parameter("compressed_image_pub_topic_name").as_string();
        std::string log_filedir_prefix = this->get_parameter("log_filedir_prefix").as_string();
        this->get_parameter("output_h", output_h);
        this->get_parameter("output_w", output_w);
        this->get_parameter("rect_x", rect_x);
        this->get_parameter("rect_y", rect_y);
        this->get_parameter("rect_h", rect_h);
        this->get_parameter("rect_w", rect_w);
        this->get_parameter("frame_rate", frame_rate);

        Size image_dsize = Size(output_w, output_h);
        Rect image_rect(rect_x, rect_y, rect_w, rect_h);

        image_pub = this->create_publisher<sensor_msgs::msg::Image>(image_pub_topic_name, 10);
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_image_pub_topic_name, 10);
        // auto image_transport=image_pub_(std::make_shared<image_transport::CameraPublisher>(
        //     image_transport::create_camera_publisher(this, "image_raw",
        //                                              rclcpp::QoS{100}.get_rmw_qos_profile())));
        time_t log_time = time(0);
        char log_time_tmp[32] = {NULL};
        strftime(log_time_tmp, sizeof(log_time_tmp), "%Y-%m-%d_%H-%M-%S", localtime(&log_time));
        const std::string log_filepathname = log_filedir_prefix + log_time_tmp + "/camera_ros.log";
        logger = spdlog::basic_logger_mt("basic_logger", log_filepathname);

        thread t1(readRTSPVideo, image_dsize, image_rect, url);
        rclcpp::WallRate loop_rate(frame_rate);
        while (rclcpp::ok())
        {
            frame_mutex.lock();
            current_frame = frame;
            frame_mutex.unlock();

            if (!current_frame.empty()) // 110ms-240ms
            {
                sensor_msgs::msg::Image current_msg;


                vector<uchar> jpeg_data;
                imencode(".jpg", current_frame, jpeg_data);
                sensor_msgs::msg::CompressedImage compressed_msg;
                compressed_msg.format = "jpeg";
                compressed_msg.data = jpeg_data;
                publisher_->publish(compressed_msg);
                // cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", current_frame).toImageMsg(current_msg);
                cv_bridge::CvImage(std_msgs::msg::Header(), "8UC3", current_frame).toImageMsg(current_msg);
                auto stamp = this->get_clock()->now();
                // double time_s = this->get_clock()->now().seconds();
                // double time_ns = this->get_clock()->now().nanoseconds();
                current_msg.header.stamp = stamp;
                // current_msg.header.stamp.sec = stamp.seconds();
                // current_msg.header.stamp.nanosec = stamp.nanoseconds();
                // std::cerr<< "time_s:  " << stamp.seconds() << "  time_ns: " << stamp.nanoseconds() << std::endl;  
                image_pub->publish(current_msg);
                // image_transport->publish(current_msg);
            }
            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
            loop_rate.sleep();
        }
    }

private:
    int rect_x, rect_y, rect_h, rect_w, output_h, output_w, frame_rate;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;

    // std::shared_ptr<image_transport::CameraPublisher> image_pub_; ???
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
