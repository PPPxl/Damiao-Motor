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
using namespace std;
/**
 * @brief The class to remote a single motor or all motors.
 * 
 */
class pdtMotor
{
private:
    int ret;
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    
public:
    pdtMotor(char *port, vector<int> ids);
    ~pdtMotor();
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    int float_to_uint(float x, float x_min, float x_max, int bits);
    void ctrl_motor(can_frame frame, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
    void ctrl_motor2(can_frame frame, uint16_t id, float _pos, float _vel);
    void ctrl_motor3(can_frame frame, uint16_t id, float _vel);
    void CAN_Receive();
    void Raspberry_CAN_RxCpltCallback(can_frame _frame);
};

/*typedef struct CAN_Handle
{
    struct pTxMsg;
}*CAN_HandleTypeDef;*/

/*struct pTxMsg
{
    string StdId;
    string IDE;
    string RTR;
    int DLC;
    double[] data = new double[10];
};*/

