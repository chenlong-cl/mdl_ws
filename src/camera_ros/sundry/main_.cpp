#include <chrono>
#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

class RTSPNode
{
public:
    RTSPNode(const std::string &rtsp_url)
        : it_(nh_)
    {
        cap_.open(rtsp_url);
        if (!cap_.isOpened())
        {
            throw std::runtime_error("Failed to open RTSP stream");
        }

        pub_ = it_.advertise("camera/image", 1);
    }

    void run()
    {
        while (rclcpp::ok())
        {
            cv::Mat frame;
            if (cap_.read(frame))
            {
                std::thread t1(&RTSPNode::processImage, this, frame.clone());
                t1.detach();
            }

            std::this_thread::sleep_for(10ms);
        }
    }

private:
    void processImage(const cv::Mat &image)
    {
        // Perform image processing operations here
        // ...

        // Publish the image as a ROS message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub_.publish(msg);
    }

    rclcpp::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " RTSP_URL" << std::endl;
        return 1;
    }

    RTSPNode node(argv[1]);
    node.run();

    rclcpp::shutdown();
    return 0;
}