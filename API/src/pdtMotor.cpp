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
 * @brief 构造函数及初始化
 * 
 * @param port   the CAN port you need to use
 * @param ids   all IDs of the motors needed controlled
 */
pdtMotor::pdtMotor(char *port, vector<int>ids)
{
    for(int i=0; i<ids.size(); i++)
    {
        ID.push_back(ids[i]);             //向vector尾部添加用push_back     
        present_position.push_back(0.0);
        present_velocity.push_back(0.0);
        present_torque.push_back(0.0);
    }
    MOTORNUM = ID.size();
    char port1[255] ="sudo ip link set ";
    char port2[255] =" type can bitrate 1000000";
    char port3[255] ="sudo ifconfig ";
    char port4[255] =" up";
    strcat(port1,port);      //字符串拼接函数strcat：port加到port1上
    strcat(port1,port2);
    strcat(port3,port);
    strcat(port3,port4);
    //system("sudo ip link set can0 type can bitrate 1000000");  //设置can0波特率
    //system("sudo ifconfig can0 up");   //设置can0使能
    //printf("%s", port1);
    //printf("%s", port3);
    system(port1);
    system(port3);
    //printf("this is a can send demo\r\n");
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
    system("sudo ifconfig can1 down");
}



/**
 * @brief   uint转float
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
 * @brief    float转uint
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
 * @brief  MIT控制模式发送函数
 * 
 * @param frame 
 * @param id 
 * @param _pos 
 * @param _vel 
 * @param _KP 
 * @param _KD 
 * @param _torq 
 */
void pdtMotor::MIT_ctrl_motor(float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));            //memset函数初始化
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    for(int i=0; i<MOTORNUM; i++)
    {
    frame.can_id = ID[i];
    frame.can_dlc = 0x08;
    frame.data[0] = (pos_tmp >> 8);
    frame.data[1] = pos_tmp;
    frame.data[2] = (vel_tmp >> 4);
    frame.data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>> 8);
    frame.data[4] = kp_tmp;
    frame.data[5] = (kd_tmp >> 4);
    frame.data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>> 8);
    frame.data[7] = tor_tmp;

    //printf("can_id  = 0x%X\r\n", frame.can_id);
    //printf("can_dlc = %d\r\n", frame.can_dlc);
    //int i = 0;
    //for(i = 0; i < 8; i++)
        //printf("data[%d] = %d\r\n", i, frame.data[i]);
   //6.send message
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) 
        {
            printf("Send Error ID %d!\r\n", ID[i]);
            system("sudo ifconfig can0 down");
        }
    }
   
}



/**
 * @brief  位置模式发送函数
 * 
 * @param frame 
 * @param id 
 * @param _pos 
 * @param _vel 
 */
void pdtMotor::pos_ctrl_motor(uint16_t id, float _pos, float _vel)
{
    uint8_t *pbuf,*vbuf;
    pbuf =(uint8_t*)&_pos;
    vbuf =(uint8_t*)&_vel;
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    frame.can_id =  id;
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
 * @brief  速度模式发送函数
 * 
 * @param frame 
 * @param id 
 * @param _vel 
 */
void pdtMotor::vel_ctrl_motor(uint16_t id, float _vel)
{
    uint8_t *vbuf;
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    vbuf=(uint8_t*)&_vel;
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    frame.can_id =  id;
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


/**
 * @brief Motor enable
 * 
 * @param id 
 */
void pdtMotor::enable()
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    for(int i=0; i<MOTORNUM; i++)
    {
        frame.can_id =ID[i];
        frame.can_dlc = 0x08;
        for(int g=0; g<7; g++)
        {
            frame.data[g] = 0xff;
            frame.data[7] = 0xfc;
        }      
    //6.send message
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) 
     {
        printf("Enable failed!\r\n");
        system("sudo ifconfig can0 down");
     }
     
    }
    //printf("can_id  = 0x%X\r\n", frame.can_id);
    //printf("can_dlc = %d\r\n", frame.can_dlc);

  
}


/**
 * @brief Motor disable
 * 
 */
void pdtMotor::disable()
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    //5.Set send data
    for(int i=0; i<MOTORNUM; i++)
    {  
        frame.can_id = ID[i];
        frame.can_dlc = 0x08;
        for(int g=0; g<7; g++)
            frame.data[g] = 0xff;
            frame.data[7] = 0xfd;        
    }
    //printf("can_id  = 0x%X\r\n", frame.can_id);
    //printf("can_dlc = %d\r\n", frame.can_dlc);

   //6.send message
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        printf("Disable failed!\r\n");
        system("sudo ifconfig can0 down");
    }
}




/**
 * @brief CAN接受解析函数
 * 
 * @return int 
 */
int pdtMotor::motor_state_receive()
{
    //4.Define receive rules
    struct can_filter rfilter[1];
    rfilter[0].can_id = 0x00;
    rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    //linux下的can通讯，filter方式，不按照id过滤，而接收所有id发送的数据
    //5.Receive data and exit
    memset(&recvFrame, 0, sizeof(struct can_frame));

    while(1) 
    {
        nbytes = read(s, &recvFrame, sizeof(recvFrame));
        if(nbytes > 0) 
        {
            float p_int, v_int, t_int;
            p_int=(recvFrame.data[1]<<8)|recvFrame.data[2];
            v_int=(recvFrame.data[3]<<4)|(recvFrame.data[4]>>4);
            t_int=((recvFrame.data[4]&0xF)<<8)|recvFrame.data[5];
            
            if(recvFrame.data[0]>=1)
            {
                present_position[recvFrame.data[0]-1] = uint_to_float(p_int, P_MIN, P_MAX, 16);
                present_velocity[recvFrame.data[0]-1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
                present_torque[recvFrame.data[0]-1] = uint_to_float(t_int, T_MIN, T_MAX, 12);
            }
            // printf("can_id = 0x%X\r\n", recvFrame.data[0]);
            // printf("position = %f\r\n", present_position);
            // printf("velocity = %f\r\n", present_velocity);
            // printf("torque = %f\r\n", present_torque);  
        break;
        }              
    }
  return (recvFrame.data[0]);
}


