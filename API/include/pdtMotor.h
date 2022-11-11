/**
 * @file pdtMotor.h
 * @author Xiangli Pei
 * @brief 
 * @version 0.1
 * @date 2022-10-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "iostream"
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <vector>
#include <fstream>


using namespace std;
/**
 * @brief The class to remote a single motor or all motors.
 * 
 */
class pdtMotor
{    
public:
    pdtMotor(char *port, vector<int> ids);
    ~pdtMotor();
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    int float_to_uint(float x, float x_min, float x_max, int bits);
    void MIT_ctrl_motor(vector<float> _pos, vector<float> _vel, vector<float> _KP, vector<float> _KD, vector<float> _torq);
    void pos_ctrl_motor(vector<float> _pos, vector<float> _vel);
    void vel_ctrl_motor(vector<float> _vel);
    int motor_state_receive();
    void enable();
    void disable();

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
    int ret;
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame recvFrame;
    vector<int> ID;
    int MOTORNUM;
    vector<float> present_position, present_velocity, present_torque;
};


