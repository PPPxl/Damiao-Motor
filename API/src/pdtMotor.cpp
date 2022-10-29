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
#include <vector>
using namespace std;


/**
 * @brief Construct a new pdt Motor::pdt Motor object
 * 
 * @param port   the CAN port you need to use
 * @param ids   all IDs of the motors needed controlled
 */
pdtMotor::pdtMotor(char *port, vector<int> ids)
{
    memset(&frame, 0, sizeof(struct can_frame));

    system("sudo ip link set can0 type can bitrate 100000");
    system("sudo ifconfig can0 up");
    printf("this is a can send demo\r\n");
     //1.Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket PF_CAN failed");
        return 1;
    }
    
    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        perror("ioctl failed");
        return 1;
    }
    
    //3.Bind the socket to can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        perror("bind failed");
        return 1;
    }
    
    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
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
void pdtMotor::ctrl_motor(can_frame frame, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    int P_MIN, P_MAX, V_MIN, V_MAX, KP_MIN, KP_MAX, KD_MIN, KD_MAX, T_MIN, T_MAX;
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
    frame.can_id = id;
    frame.can_dlc = 0x08;
    frame.data[0] = (pos_tmp >> 8);
    frame.data[1] = pos_tmp;
    frame.data[2] = (vel_tmp >> 4);
    frame.data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>> 8);
    frame.data[4] = kp_tmp;
    frame.data[5] = (kd_tmp >> 4);
    frame.data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>> 8);
    frame.data[7] = tor_tmp;

    int s,nbytes;
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
void pdtMotor::ctrl_motor2(can_frame frame, uint16_t id, float _pos, float _vel)
{
    uint8_t *pbuf,*vbuf;
    pbuf =(uint8_t*)&_pos;
    vbuf =(uint8_t*)&_vel;
    frame.can_id = id;
    frame.can_dlc = 0x08;
    frame.data[0] = *pbuf;
    frame.data[1] = *(pbuf+1);
    frame.data[2] = *(pbuf+2);
    frame.data[3] = *(pbuf+3);
    frame.data[4] = *vbuf;
    frame.data[5] = *(vbuf+1);
    frame.data[6] = *(vbuf+2);
    frame.data[7] = *(vbuf+3);

    int s,nbytes;
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
void pdtMotor::ctrl_motor3(can_frame frame, uint16_t id, float _vel)
{
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;
    frame.can_id = id;
    frame.can_dlc = 0x04;
    frame.data[0] = *vbuf;
    frame.data[1] = *(vbuf+1);
    frame.data[2] = *(vbuf+2);
    frame.data[3] = *(vbuf+3);

    int s,nbytes;
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        printf("Send Error frame[0]!\r\n");
        system("sudo ifconfig can0 down");
    }
}



/**
 * @brief  receive and analytic function for the three modes of the motor
 * 
 * @param _frame 
 * @note  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!unfinished
 */
void pdtMotor::HAL_CAN_RxCpltCallback(can_frame _frame)
{
    if(HAL_GetTick() - FlashTimer>500)
    {
        HAL_GPIO_TogglePin()
    }
}


