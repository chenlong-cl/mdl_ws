#include <iostream>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

mutex frame_mutex;
Mat frame;
std::string rtsp_url = "rtsp://admin:bdtd123456@192.168.1.39:554/h265/ch1/main/av_stream";

void readRTSPVideo()
{
    VideoCapture cap(rtsp_url);

    while (true)
    {
        Mat temp;
        cap >> temp;

        frame_mutex.lock();
        frame = temp.clone();
        frame_mutex.unlock();
    }
}

int main()
{
    thread t1(readRTSPVideo);

    while (true)
    {
        Mat current_frame;

        frame_mutex.lock();
        current_frame = frame.clone();
        frame_mutex.unlock();

        if (!current_frame.empty())
        {
            imshow("RTSP Video Stream", current_frame);
            waitKey(1);
        }
    }

    return 0;
}