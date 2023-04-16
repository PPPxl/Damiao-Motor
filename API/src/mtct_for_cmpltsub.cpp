#include <iostream>
#include <pdtMotor.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>
#include <algorithm>
using namespace std;



int main( int argc, char** argv )
{
vector<int> ids;
ids.push_back(4);
pdtMotor motorCtl("can1", ids); 

vector<float> target_pos;
vector<float> target_vel;
vector<float> target_KP;
vector<float> target_KD;
vector<float> target_tor;

target_pos.push_back(0.0);
target_vel.push_back(0.0);
target_KP.push_back(1.0);
target_KD.push_back(0.1);
target_tor.push_back(0.0);
  
motorCtl.enable();
motorCtl.MIT_ctrl_motor(target_pos, target_vel, target_KP, target_KD, target_tor); // 回零。
usleep(30e5);
float tmpTorque=0;
for(int i=0; i<10; i++)
{
  motorCtl.MIT_ctrl_motor(target_pos, target_vel, target_KP, target_KD, target_tor); //
  int updatedID = motorCtl.motor_state_receive(); //获得电机状态---------
  tmpTorque += motorCtl.present_torque[0];
  usleep(1.5e3);
}

float init_tor = 0;
// float init_tor = tmpTorque/10;
float K = 1.0;  // 1.0
float D = 0.02;  // -0.05
float tp = 0;
float tv = 0;
// 以上初始化
float k = 0.0;
// ofstream ofs1, ofs2, ofs3, ofs4;  
// ofs1.open("pos.txt",ios::out);
// ofs2.open("vel.txt",ios::out);
// ofs3.open("tor.txt",ios::out);
// ofs4.open("timeuse1.txt",ios::out);

while(1)
  {
    //send data to motor
    struct timeval startTime1, endTime1;
    double timeUse1;

    // float pos0, vel0;
    // k += 1;
    // pos0 = sin(k/1000*3.1415926/2);
    // vel0 = cos(k/1000*3.1415926/2)*(1.0/1000*3.1415926/2);
    //
    target_pos[0] = 0;
    target_vel[0] = 0;
    target_KP[0] = 0.0;
    target_KD[0] = 0.0;
    // target_tor[0] = 0.0;

    motorCtl.MIT_ctrl_motor(target_pos, target_vel, target_KP, target_KD, target_tor);


    //receive motor data
    // struct timeval startTime,endTime;
    // double timeUse;
    // gettimeofday(&startTime,NULL);
    int updatedID = motorCtl.motor_state_receive();
    // gettimeofday(&endTime,NULL);  
    // timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;

    float pp = motorCtl.present_position[0];
    float pv = motorCtl.present_velocity[0];
    target_tor[0] = init_tor * sin(pp) + K * (tp - pp) + D * (tv - pv); //确定正负号-------------------

    auto it = find(ids.begin(), ids.end(), 4);  //查找ID号4
    if ( it == ids.end( ) ) //没找到
        cout << "ERROR:find the motor ID 4" << endl;
    int bitNumber = distance(ids.begin(), it); 
    cout <<updatedID<<","<<"Pos: "<<motorCtl.present_position[bitNumber]<<endl;
    cout <<updatedID<<","<<"Vel: "<<motorCtl.present_velocity[bitNumber]<<endl;
    usleep(5000);
  } 
}
