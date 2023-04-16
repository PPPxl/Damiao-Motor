#include <iostream>
#include <pdtMotor.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
using namespace std;

float P_MIN = -12.5;
float P_MAX = 12.5;
float V_MIN = -30.0;
float V_MAX = 30.0;
float KP_MIN = 0.0;
float KP_MAX = 500.0;
float KD_MIN = 0.0;
float KD_MAX = 5.0;
float T_MIN = -18.0;
float T_MAX = 18.0;
int main(int argc, char** argv)
{
    system("sudo ip link set can1 type can bitrate 1000000");
    system("sudo ifconfig can1 up");
    cout<<"Recv init done..."<<endl;
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket PF_CAN failed");
    struct can_filter rfilter[1];
    struct can_frame recvFrame;
    rfilter[0].can_id = 0x00;      //0x00 is the master ID for receive the feedback value
    rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    //linux下的can通讯，filter方式，不按照id过滤，接收所有id发送的数据
    //5.Receive data and exit
    memset(&recvFrame, 0, sizeof(struct can_frame));
    cout<<"Recv init done..."<<endl;
    while(1) 
    {
        int nbytes = read(s, &recvFrame, sizeof(recvFrame));
        cout<<"Recv0..."<<endl;
        if(nbytes > 0) 
        {
            float p_int, v_int, t_int;
            p_int = (recvFrame.data[1]<<8)|recvFrame.data[2];
            v_int = (recvFrame.data[3]<<4)|(recvFrame.data[4]>>4);
            t_int = ((recvFrame.data[4]&0xF)<<8)|recvFrame.data[5];
            cout<<"Recv1..."<<endl;
            // printf("can_id = 0x%X\r\n", recvFrame.data[0]);
            // printf("position = %f\r\n", present_position);
            // printf("velocity = %f\r\n", present_velocity);
            // printf("torque = %f\r\n", present_torque);  
        break;
        } 

    }
}
}
