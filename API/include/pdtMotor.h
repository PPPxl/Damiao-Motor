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

#define CAN_EFF_FLAG 0x80000000U //扩展帧的标识
// #define CAN_RTR_FLAG 0x40000000U //远程帧的标识
#define CAN_RTR_FLAG 0x00 //远程帧的标识
#define CAN_ERR_FLAG 0x20000000U //错误帧的标识，用于错误检查

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
    struct can_frame recvFrame;
    
public:
    pdtMotor(char *port, vector<int> ids);
    ~pdtMotor();
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    int float_to_uint(float x, float x_min, float x_max, int bits);
    void ctrl_motor(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
    void ctrl_motor2(uint16_t id, float _pos, float _vel);
    void ctrl_motor3(uint16_t id, float _vel);
    void CAN_Receive();
    void Raspberry_CAN_RxCpltCallback(can_frame _frame);
    void enable(uint16_t id);
    void disable(uint16_t id);
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

