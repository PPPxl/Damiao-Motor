/**
 * @file pdtMotor.cpp
 * @author Xiangli Pei
 * @brief 
 * @version 0.1
 * @date 2022-10-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "pdtMotor.h"
using namespace std;


/**
 * @brief Construct a new pdt Motor::pdt Motor object
 * 
 * @param port   the CAN port you need to use
 * @param ids   all IDs of the motors needed controlled
 */
pdtMotor::pdtMotor(char *port, vector<int> ids)
{
    
    char port1[255] ="sudo ip link set ";
    char port2[255] =" type can bitrate 1000000";
    char port3[255] ="sudo ifconfig ";
    char port4[255] =" up";
    strcat(port1,port);   //字符串的拼接：port加到port1上
    strcat(port1,port2);
    strcat(port3,port);
    strcat(port3,port4);
    //system("sudo ip link set can0 type can bitrate 1000000");
    //system("sudo ifconfig can0 up");
    printf("%s", port1);
    printf("%s", port3);
    system(port1);
    system(port3);
    printf("this is a can send demo\r\n");
     //1.Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket PF_CAN failed");
      //  return 1;
    }
    
    //2.Specify can0 device
    strcpy(ifr.ifr_name, port);
   // strcpy(ifr.ifr_name, port);
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        perror("ioctl failed");
      //  return 1;
    }
    
    //3.Bind the socket to can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        perror("bind failed");
      //  return 1;
    }  
}




/**
 * @brief Destroy the pdt Motor::pdt Motor object
 * 
 */
pdtMotor::~pdtMotor()
{
    close(s);
    system("sudo ifconfig can0 down");
}


/**
 * @brief   uint to float
 * 
 * @param x_int 
 * @param x_min 
 * @param x_max 
 * @param bits 
 * @return float 
 */
float pdtMotor::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}



/**
 * @brief    float to uint
 * 
 * @param x 
 * @param x_min 
 * @param x_max 
 * @param bits 
 * @return int 
 */
int pdtMotor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}



/**
 * @brief  send function of MIT control mode for the motor
 * 
 * @param frame 
 * @param id 
 * @param _pos 
 * @param _vel 
 * @param _KP 
 * @param _KD 
 * @param _torq 
 */
void pdtMotor::ctrl_motor(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    int P_MIN, P_MAX, V_MIN, V_MAX, KP_MIN, KP_MAX, KD_MIN, KD_MAX, T_MIN, T_MAX;
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    frame.can_id = CAN_RTR_FLAG | id ;
    frame.can_dlc = 0x08;
    frame.data[0] = (pos_tmp >> 8);
    frame.data[1] = pos_tmp;
    frame.data[2] = (vel_tmp >> 4);
    frame.data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>> 8);
    frame.data[4] = kp_tmp;
    frame.data[5] = (kd_tmp >> 4);
    frame.data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>> 8);
    frame.data[7] = tor_tmp;

    printf("can_id  = 0x%X\r\n", frame.can_id);
    printf("can_dlc = %d\r\n", frame.can_dlc);
    int i = 0;
    for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);
   //6.send message
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        printf("Send Error frame[0]!\r\n");
        system("sudo ifconfig can0 down");
    }
}



/**
 * @brief  send function of Postion control mode for the motor
 * 
 * @param frame 
 * @param id 
 * @param _pos 
 * @param _vel 
 */
void pdtMotor::ctrl_motor2(uint16_t id, float _pos, float _vel)
{
    uint8_t *pbuf,*vbuf;
    pbuf =(uint8_t*)&_pos;
    vbuf =(uint8_t*)&_vel;
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    frame.can_id = CAN_RTR_FLAG | id;
    frame.can_dlc = 0x08;
    frame.data[0] = *pbuf;
    frame.data[1] = *(pbuf+1);
    frame.data[2] = *(pbuf+2);
    frame.data[3] = *(pbuf+3);
    frame.data[4] = *vbuf;
    frame.data[5] = *(vbuf+1);
    frame.data[6] = *(vbuf+2);
    frame.data[7] = *(vbuf+3);

    printf("can_id  = 0x%X\r\n", frame.can_id);
    printf("can_dlc = %d\r\n", frame.can_dlc);
    int i = 0;
    for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);
   //6.send message
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        printf("Send Error frame[0]!\r\n");
        system("sudo ifconfig can0 down");
    }
}



/**
 * @brief  send function of Speed control mode for the motor
 * 
 * @param frame 
 * @param id 
 * @param _vel 
 */
void pdtMotor::ctrl_motor3(uint16_t id, float _vel)
{
    uint8_t *vbuf;
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    vbuf=(uint8_t*)&_vel;
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    frame.can_id = CAN_RTR_FLAG | id;
    frame.can_dlc = 0x04;
    frame.data[0] = *vbuf;
    frame.data[1] = *(vbuf+1);
    frame.data[2] = *(vbuf+2);
    frame.data[3] = *(vbuf+3);

    printf("can_id  = 0x%X\r\n", frame.can_id);
    printf("can_dlc = %d\r\n", frame.can_dlc);
    int i = 0;
    for(i = 0; i < 8; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);
   //6.send message
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        printf("Send Error frame[0]!\r\n");
        system("sudo ifconfig can0 down");
    }
}

void pdtMotor::enable(uint16_t id)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    frame.can_id = CAN_RTR_FLAG | id ;
    frame.can_dlc = 0x08;
    for(int i=0; i<7; i++)
    frame.data[i] = 0xff;
    frame.data[7] = 0xfc;

    printf("can_id  = 0x%X\r\n", frame.can_id);
    printf("can_dlc = %d\r\n", frame.can_dlc);

   //6.send message
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        printf("Enable failed!\r\n");
        system("sudo ifconfig can0 down");
    }
}

void pdtMotor::disable(uint16_t id)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    frame.can_id = CAN_RTR_FLAG | id ;
    frame.can_dlc = 0x08;
    for(int i=0; i<7; i++)
    frame.data[i] = 0xff;
    frame.data[7] = 0xfd;

    printf("can_id  = 0x%X\r\n", frame.can_id);
    printf("can_dlc = %d\r\n", frame.can_dlc);

   //6.send message
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        printf("Disable failed!\r\n");
        system("sudo ifconfig can0 down");
    }
}

/**
 * @brief  receive and analytic function for the three modes of the motor接收解析函数
 * 
 * @param _frame 
 * @note  
 */
void pdtMotor::Raspberry_CAN_RxCpltCallback(can_frame _frame)
{

    //ignore can1 or can2
    if(_frame.can_id == 0)
    {
        int p_int, v_int, t_int, P_MIN, P_MAX, V_MIN, V_MAX, T_MIN, T_MAX;
        p_int=(_frame.data[1]<<8)|_frame.data[2];
        v_int=(_frame.data[3]<<4)|(_frame.data[4]>>4);
        t_int=((_frame.data[4]&0xF)<<8)|_frame.data[5];
        float position, velocity, torque;
        position = uint_to_float(p_int, P_MIN, P_MAX, 16);
        velocity = uint_to_float(v_int, V_MIN, V_MAX, 16);
        torque = uint_to_float(t_int, T_MIN, T_MAX, 16);
    }
}


/**
 * @brief 
 * 
 */
void pdtMotor::CAN_Receive()
{
     //4.Define receive rules
    // struct can_filter rfilter[1];
    // rfilter[0].can_id = 0x01;
    // rfilter[0].can_mask = CAN_SFF_MASK;
    // setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
   //linux下的can通讯，filter方式，不按照id过滤，而是接收所有id发送的数据
    //5.Receive data and exit
    memset(&recvFrame, 0, sizeof(struct can_frame));
    while(1) {
        nbytes = read(s, &recvFrame, sizeof(recvFrame));
        if(nbytes > 0) {
            printf("can_id = 0x%X\r\ncan_dlc = %d \r\n", recvFrame.can_id, recvFrame.can_dlc);
            int i = 0;
            for(i = 0; i < 8; i++)
                printf("data[%d] = %d\r\n", i, recvFrame.data[i]);
            break;
        }
    }
}

