#include <iostream>
#include <pdtMotor.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>
using namespace std;



int main( int argc, char** argv )
{
vector<int> ids;
ids.push_back(1);
pdtMotor motorCtl("can1", ids); 

vector<float> target_pos;
vector<float> target_vel;
vector<float> target_KP;
vector<float> target_KD;
vector<float> targrt_tor;

target_pos.push_back(0.0);
target_vel.push_back(0.0);
target_KP.push_back(0.18);
target_KD.push_back(0.00);
targrt_tor.push_back(0.0);
  
motorCtl.enable();
motorCtl.MIT_ctrl_motor(target_pos, target_vel, target_KP, target_KD, targrt_tor);
// usleep(20e6);
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

    float pos0, vel0;
    k += 1;
    pos0 = sin(k/1000*3.1415926/2);
    vel0 = cos(k/1000*3.1415926/2)*(1.0/1000*3.1415926/2);
    //
    // target_pos[0] = 0;
    // target_vel[0] = 0;
    // target_KP[0] = 0.02;
    // target_KD[0] = 0.0;
    // targrt_tor[0] = 0.0;

    motorCtl.MIT_ctrl_motor(target_pos, target_vel, target_KP, target_KD, targrt_tor);
    // usleep(30);

    //receive motor data
    // struct timeval startTime,endTime;
    // double timeUse;
    // gettimeofday(&startTime,NULL);
    int updatedID = motorCtl.motor_state_receive();
    // gettimeofday(&endTime,NULL);  
    // timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
    cout <<updatedID<<","<<"Pos: "<<motorCtl.present_position[0]<<endl;
    cout <<updatedID<<","<<"Vel: "<<motorCtl.present_velocity[0]<<endl;

    usleep(5000);
  } 
}
