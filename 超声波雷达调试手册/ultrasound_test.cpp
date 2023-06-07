#include <stdio.h>
#include <fcntl.h>
#include <termios.h> 
#include <unistd.h>
#include <string>
#include <errno.h>
#include <stdlib.h>
#include <chrono>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <list>
#define CLOCKS_PER_SEC ((clock_t)1000)
using namespace std;

int main(void)
{
    
    int fd0,fd1; /*File Descriptor*/
    list<float> ultrasound_distance;
    printf("\n +----------------------------------+");
    printf("\n |        Serial Port Write         |");
    printf("\n +----------------------------------+");

    fd0 = open("/dev/ttyCH9344USB1", O_RDWR | O_NOCTTY | O_NDELAY);
    fd1 = open("/dev/ttyCH9344USB5", O_RDWR | O_NOCTTY | O_NDELAY);

    fcntl(fd0,F_SETFL,0);
    fcntl(fd1,F_SETFL,0);

    if (fd0 == -1) /* Error Checking */
        printf("\n  Error! in Opening ttyCH9344USB1");
    else
        printf("\n  ttyCH9344USB1 Opened Successfully ");
    if (fd1 == -1) /* Error Checking */
        printf("\n  Error! in Opening ttyCH9344USB5");
    else
        printf("\n   ttyCH9344USB5 Opened Successfully ");
    struct termios oldtio,SerialPortSettings; 

    tcgetattr(fd0, &oldtio); 
    tcgetattr(fd1, &oldtio); 

    //cfmakeraw(&SerialPortSettings);
    bzero(&SerialPortSettings,sizeof(SerialPortSettings));
    //tcflush(fd0,TCIOFLUSH);
    //tcflush(fd1,TCIOFLUSH);

    //设置波特率
    cfsetispeed(&SerialPortSettings, B9600); 
    cfsetospeed(&SerialPortSettings, B9600); 
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; 
    SerialPortSettings.c_cc[VTIME]=10;
    SerialPortSettings.c_cc[VMIN]=24;
    tcflush(fd0,TCIOFLUSH);
    tcsetattr(fd0, TCSANOW, &SerialPortSettings);
    tcsetattr(fd1, TCSANOW, &SerialPortSettings);

    //设置没有校验
    /*SerialPortSettings.c_cflag &= ~PARENB; 

    //停止位 = 1
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE; 

    //设置数据位 = 8
    SerialPortSettings.c_cflag |= CS8;    

    SerialPortSettings.c_cflag &= ~CRTSCTS;      
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; 

    //关闭软件流动控制
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);  

    //设置操作模式    
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); 

    SerialPortSettings.c_oflag &= ~OPOST;

    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0)
        printf("\n  ERROR ! in Setting attributes");
    else
        printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none");
    */
    //定义传输内容
    char write_buffer0[8] = {0xAA,0xAA,0xAA,0xA5,0x01,0xAF};
    char write_buffer1[8] = {0xAA,0xAA,0xAA,0xA5,0x01,0xAF};
    char buffer0[24];
    char buffer1[24];
    //传输字节数 
    int bytes_written0 = 0;
    int bytes_written1 = 0;

    int bytes_read0=0;
    int bytes_read1=0;
    //size_t n = fd.available();
    
    //printf("%f ", duration);
    while(1)
    {

    //串口写数据
    bytes_written0 = write(fd0, write_buffer0, sizeof(write_buffer0));
    bytes_written1 = write(fd1, write_buffer1, sizeof(write_buffer1));  
    //tcdrain(fd0);
    //tcdrain(fd1);                                                                                                   
    printf("\n  %s written to  ttyCH9344USB1", write_buffer0);
    printf("\n  %d Bytes written to ttyCH9344USB1", bytes_written0);
    printf("\n  %s written to  ttyCH9344USB5", write_buffer1);
    printf("\n  %d Bytes written to ttyCH9344USB5", bytes_written1);
    printf("\n +----------------------------------+\n\n");
    sleep(0.5);
    auto start = std::chrono::system_clock::now();
    if(bytes_written0!=0 && bytes_written1!=0)
        {
            //读出数据
            bytes_read0=read(fd0,buffer0,sizeof(buffer0));
            bytes_read1=read(fd1,buffer1,sizeof(buffer1));
            auto end = std::chrono::system_clock::now();
            std::cerr <<"串口 seconds"<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
            std::cerr <<bytes_read0<< std::endl;
            std::cerr <<bytes_read1<< std::endl;
          float front_distance1=buffer0[4]*0.1;
          float front_distance2=buffer0[10]*0.1;
          float front_distance3=buffer0[16]*0.1;
          float front_distance4=buffer0[22]*0.1;
          
          float back_distance1=buffer1[4]*0.1;
          float back_distance2=buffer1[10]*0.1;
          float back_distance3=buffer1[16]*0.1;
          float back_distance4=buffer1[22]*0.1;

          ultrasound_distance.emplace_back(front_distance1);
          ultrasound_distance.emplace_back(front_distance2);
          ultrasound_distance.emplace_back(front_distance3);
          ultrasound_distance.emplace_back(front_distance4);

          ultrasound_distance.emplace_back(back_distance1);
          ultrasound_distance.emplace_back(back_distance2);
          ultrasound_distance.emplace_back(back_distance3);
          ultrasound_distance.emplace_back(back_distance4);

            ultrasound_distance.sort();
 
            float min_distance=ultrasound_distance.front();
            printf("min distance: %.2f\n ", min_distance);
            if (min_distance > 1.8 && min_distance <= 2.3)
          {//frame[1].data[0] =0x11;
          std::cerr<<"超声波雷达提示"<<std::endl;}
        else if (min_distance > 1.3 && min_distance <= 1.8)
          {//frame[1].data[0] =0x22;
          std::cerr<<"超声波雷达预警"<<std::endl;}
        else if (min_distance > 0.8 && min_distance <= 1.3)
          {//frame[1].data[0] =0x33;
          std::cerr<<"超声波雷达报警"<<std::endl;}
        else if (min_distance >= 0.3 && min_distance <= 0.8)
          {//frame[1].data[0] =0x44;
          std::cerr<<"超声波雷达制动"<<std::endl;}
            for(int i=0; i<bytes_read0; i++)
            {
                //16进制的方式打印到屏幕
                //string hex_16=std::to_string(buffer[i] & 0xff);
                int int_10=buffer0[i];
                //std::cerr << std::hex << (buffer[i] & 0xff) << " ";
                printf("%02X ", buffer0[i]);
                //printf("%d ", int_10);
            }
            for(int i=0; i<bytes_read1; i++)
            {
                //16进制的方式打印到屏幕
                //string hex_16=std::to_string(buffer[i] & 0xff);
                int int_10=buffer1[i];
                //std::cerr << std::hex << (buffer[i] & 0xff) << " ";
                printf("%02X ", buffer1[i]);
                printf("%d ", int_10);
            }
            std::cerr << std::endl;
            memset(buffer0,0,sizeof(buffer0));
        }
        ultrasound_distance.clear();
        
    
    }
    close(fd0); 
    close(fd1); 

    return 0;
}

