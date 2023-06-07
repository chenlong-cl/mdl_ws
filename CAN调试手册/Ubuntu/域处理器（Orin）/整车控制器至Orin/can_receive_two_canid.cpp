
/* 1. 报文接收程序 */

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
    int s, nbytes1,nbytes2;
	struct sockaddr_can addr;
    struct ifreq ifr;

    struct can_frame frame[2] = {{0}};
    //struct can_frame frame;
    struct can_filter rfilter[2];
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
    strcpy(ifr.ifr_name,"can1");
    ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与 can0 绑定

    //定义接收规则，只接收表示符等于 0x11 的报文
    rfilter[0].can_id = 0x180028D0;
    rfilter[1].can_id = 0x180128D0;

    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_mask = CAN_SFF_MASK;

    //设置过滤规则
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

	while(1)
    {

        nbytes1 = read(s, &frame[0], sizeof(frame[0])); //接收报文

        if(nbytes1 > 0)
        {
            printf("ID=0x%X DLC=%d data[0]=0x%X data[1]=0x%X data[2]=0x%X data[3]=0x%X data[4]=0x%X data[5]=0x%X data[6]=0x%X data[7]=0x%X\n", frame[0].can_id,
                frame[0].can_dlc, frame[0].data[0],frame[0].data[1],frame[0].data[2],frame[0].data[3],frame[0].data[4],frame[0].data[5],frame[0].data[6],frame[0].data[7]);

        }
        nbytes2 = read(s, &frame[1], sizeof(frame[1])); //接收报文

        if(nbytes2 > 0)
        {
            printf("ID=0x%X DLC=%d data[0]=0x%X data[1]=0x%X data[2]=0x%X data[3]=0x%X data[4]=0x%X data[5]=0x%X data[6]=0x%X data[7]=0x%X\n", frame[1].can_id,
                frame[1].can_dlc, frame[1].data[0],frame[1].data[1],frame[1].data[2],frame[1].data[3],frame[1].data[4],frame[1].data[5],frame[1].data[6],frame[1].data[7]);

        }
        
    }
    close(s);
    return 0;

}
