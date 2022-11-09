#include <iostream>
#include <pdtMotor.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>
using namespace std;


void *ctrlSend(void *data)
{
vector<int> ID;
pdtMotor MotorSend("can1", ID);
//struct can_frame frame;
uint16_t id1 = 1;
uint16_t id2 = 2;
MotorSend.enable();

//for(int i=0; i<=50; i++)
float k = 0.0;
MotorSend.MIT_ctrl_motor(0, 0, 0.05, 0.05, 0);
MotorSend.MIT_ctrl_motor(0, 0, 0.05, 0.05, 0);
usleep(1.5e6);
ofstream ofs4, ofs5, ofs6;  
    ofs4.open("timeuse1.txt",ios::out);
    ofs5.open("timeuse2.txt",ios::out);
    ofs6.open("timeuse3.txt",ios::out);
while(1)
  {
    
    struct timeval startTime,endTime1,endTime2;
    double timeUse1,timeUse2,timeUse3;
    gettimeofday(&startTime,NULL);
     float pos1, pos2, vel1, vel2;
   
      k += 1;
      pos1 = sin(k/1000*3.1415926/2);
      vel1 = cos(k/1000*3.1415926/2)*(1.0/1000*3.1415926/2);
      pos2 = sin(k/1000*3.1415926/2);
      vel2 = cos(k/1000*3.1415926/2)*(1.0/1000*3.1415926/2);   
 
    MotorSend.MIT_ctrl_motor(pos1, vel1, 5, 0.5, 0);
    gettimeofday(&endTime1,NULL);  
    timeUse1 = 1e6*(endTime1.tv_sec - startTime.tv_sec) + endTime1.tv_usec - startTime.tv_usec; 
    //printf("timeUse1 = %f\r\n",timeUse1);
    ofs4 <<timeUse1<<endl;
    usleep(4e2);
    MotorSend.MIT_ctrl_motor(0, 0, 1, 0.1, 0);
    gettimeofday(&endTime2,NULL);
    timeUse2 = 1e6*(endTime2.tv_sec - startTime.tv_sec) + endTime2.tv_usec - startTime.tv_usec;
    //printf("timeUse2 = %f\r\n",timeUse2);
    ofs5 <<timeUse2<<endl;    
    timeUse3 = timeUse2-timeUse1;
    //printf("timeUse3 = %f\r\n",timeUse3);
    ofs6 <<timeUse3<<endl;
    usleep(4e2);
  }


}


void *ctrlReceive(void *data)
{
vector<int> ID;
ID.push_back(1);
ID.push_back(2);
pdtMotor MotorReceive("can1", ID);

ofstream ofs1, ofs2, ofs3;  
ofs1.open("pos.txt",ios::out);
ofs2.open("vel.txt",ios::out);
ofs3.open("tor.txt",ios::out);
struct timeval startTime,endTime;
double timeUse;
gettimeofday(&startTime,NULL);
while(1)
{
  int updatedID = MotorReceive.motor_state_receive();
  gettimeofday(&endTime,NULL);  
  timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
  ofs1 <<updatedID<<","<<timeUse<<","<<MotorReceive.present_position[updatedID-1]<<endl;
  ofs2 <<updatedID<<","<<timeUse<<","<<MotorReceive.present_velocity[updatedID-1]<<endl;
  ofs3 <<updatedID<<","<<timeUse<<","<<MotorReceive.present_torque[updatedID-1]<<endl;
}
ofs1.close();  
ofs2.close();  
ofs3.close(); 

}



int main( int argc, char** argv )
{

pthread_t th1, th2;
int ret;
ret = pthread_create(&th1,NULL,ctrlSend,NULL);
if(ret != 0)
{
	printf("create pthread1 error!\n");
	exit(1);
}
ret = pthread_create(&th2,NULL,ctrlReceive,NULL);
if(ret != 0)
{
		printf("create pthread2 error!\n");
		exit(1);
}

pthread_join(th1, NULL);
pthread_join(th2, NULL);
while(1);

return 0;
}
