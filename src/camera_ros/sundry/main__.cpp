#include <iostream>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

mutex frame_mutex;
Mat frame;

void readRTSPVideo()
{
    VideoCapture cap("rtsp://example.com/stream"); // RTSP视频流地址

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