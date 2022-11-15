#include <iostream>
#include <pdtMotor.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>
using namespace std;

 

void *ctrlSendandRec(void *data)
{
vector<int> ids;
ids.push_back(1);
ids.push_back(2);
pdtMotor motorCtl("can1", ids); 

  vector<float> target_pos;
  vector<float> target_vel;
  vector<float> target_KP;
  vector<float> target_KD;
  vector<float> targrt_tor;

  target_pos.push_back(0.0);
  target_vel.push_back(0.0);
  target_KP.push_back(0.5);
  target_KD.push_back(0.01);
  targrt_tor.push_back(0.0);

  target_pos.push_back(0.0);
  target_vel.push_back(0.0);
  target_KP.push_back(0.5);
  target_KD.push_back(0.01);
  targrt_tor.push_back(0.0);
  
motorCtl.enable();
motorCtl.MIT_ctrl_motor(target_pos, target_vel, target_KP, target_KD, targrt_tor);
usleep(1.5e6);
float k = 0.0;
//ofstream ofs4;           //ofstream输出文件流
//ofs4.open("timeuse1.txt",ios::out);
while(1)
  {
    //struct timeval startTime, endTime1;
    //double timeUse1;
    //gettimeofday(&startTime,NULL);
    float pos0, vel0;
    k += 1;
    pos0 = sin(k/1000*3.1415926/2);
    vel0 = cos(k/1000*3.1415926/2)*(1.0/1000*3.1415926/2);
    //
    target_pos[0] = pos0;
    target_vel[0] = vel0;
    target_KP[0] = 5.0;
    target_KD[0] = 0.5;
    targrt_tor[0] = 0.0;
    //
    target_pos[1] = pos0;
    target_vel[1] = vel0;
    target_KP[1] = 5.0;
    target_KD[1] = 0.5;
    targrt_tor[1] = 0.0;
    motorCtl.MIT_ctrl_motor(target_pos, target_vel, target_KP, target_KD, targrt_tor);
    //gettimeofday(&endTime1,NULL);  
    //timeUse1 = 1e6*(endTime1.tv_sec - startTime.tv_sec) + endTime1.tv_usec - startTime.tv_usec; 
    //printf("timeUse1 = %f\r\n",timeUse1);
    //ofs4 <<timeUse1<<endl;   
  }
  //ofs4.close();  

ofstream ofs1, ofs2, ofs3;  
ofs1.open("pos.txt",ios::out);
ofs2.open("vel.txt",ios::out);
ofs3.open("tor.txt",ios::out);
struct timeval startTime,endTime;
double timeUse;
gettimeofday(&startTime,NULL);
while(1)
{
  int updatedID = motorCtl.motor_state_receive();
  gettimeofday(&endTime,NULL);  
  timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
  ofs1 <<updatedID<<","<<timeUse<<","<<motorCtl.present_position[updatedID-1]<<endl;
  ofs2 <<updatedID<<","<<timeUse<<","<<motorCtl.present_velocity[updatedID-1]<<endl;
  ofs3 <<updatedID<<","<<timeUse<<","<<motorCtl.present_torque[updatedID-1]<<endl;
}
ofs1.close();  
ofs2.close();  
ofs3.close(); 
}


int main( int argc, char** argv )
{
pthread_t th1;
int ret;
ret = pthread_create(&th1,NULL,ctrlSendandRec,NULL);
if(ret != 0)
{
	printf("create pthread1 error!\n");
	exit(1);
}

pthread_join(th1, NULL);
while(1);

return 0;
}
