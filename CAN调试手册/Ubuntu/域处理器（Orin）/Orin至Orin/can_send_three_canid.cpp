
/* 1. 报文发送程序 */

#include <stdio.h>

#include <stdlib.h>

#include <string.h>

#include <unistd.h>

#include <net/if.h>

#include <sys/ioctl.h>

#include <sys/socket.h>

#include <linux/can.h>

#include <linux/can/raw.h>

int main()
{
    int s, nbytes0, nbytes1, nbytes2, nbytes3;
    struct sockaddr_can addr;
    struct ifreq ifr;

    struct can_frame frame[4] = {{0}};
    struct can_filter rfilter[1];
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 创建套接字
    strcpy(ifr.ifr_name, "can1");
    ioctl(s, SIOCGIFINDEX, &ifr); // 指定 can0 设备

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr)); // 将套接字与 can0 绑定

    // 禁用过滤规则，本进程不接收报文，只负责发送
    // setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    // 定义接收规则，只接收表示符等于 0x11 的报文
    rfilter[0].can_id = 0x22;
    rfilter[0].can_mask = CAN_SFF_MASK;
    // 设置过滤规则
    // setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    // 生成两个报文

    frame[0].can_id = CAN_EFF_FLAG | 0x180028D0;

    frame[0].can_dlc = 8;
    frame[0].data[0] = 0xD5;//11010101
    frame[0].data[1] = 0x5A;//90
    frame[0].data[2] = 0x50;//80
    frame[0].data[3] = 0x01;
    frame[0].data[4] = 0x2C;
    frame[0].data[5] = 0x01;
    frame[0].data[6] = 0x01;



    frame[2].can_id = CAN_EFF_FLAG | 0x180D20E0;

    frame[2].can_dlc = 8;
    frame[2].data[0] = 0x01;
    frame[2].data[1] = 0x20;
    frame[2].data[2] = 0x30;
    frame[2].data[3] = 0x40;

    frame[3].can_id = CAN_EFF_FLAG | 0x180128D0;

    frame[3].can_dlc = 8;
    frame[3].data[0] = 0x12;
    frame[3].data[1] = 0x21;
    frame[3].data[2] = 0x31;
    frame[3].data[3] = 0x41;
    // 循环发送两个报文

    while (1)
    {

        if (write(s,&frame[0],sizeof(can_frame))!=sizeof(struct can_frame)){
            printf("send frame0 error\n");
        }

        if (write(s,&frame[2],sizeof(can_frame))!=sizeof(struct can_frame)){
            printf("send frame2 error\n");}

        if (write(s,&frame[3],sizeof(can_frame))!=sizeof(struct can_frame)){
            printf("send frame3 error\n");}

        sleep(0.02);



    }
    close(s);
    return 0;
}
