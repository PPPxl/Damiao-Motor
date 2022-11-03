#include <iostream>
#include <pdtMotor.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
using namespace std;


void *ctrlSend(void *data)
{
vector<int> ID;
pdtMotor MotorSend("can1", ID);
//struct can_frame frame;
uint16_t id1 = 1;
uint16_t id2 = 2;
MotorSend.enable(id1);
MotorSend.enable(id2);
for(int i=0; i<=10; i++)
  {
    struct timeval startTime,endTime1,endTime2;
    double timeUse1,timeUse2;
    gettimeofday(&startTime,NULL);

    MotorSend.ctrl_motor(id1, 7, 0, 0.5, 0.01, 0);
    gettimeofday(&endTime1,NULL);  
    timeUse1 = 1e6*(endTime1.tv_sec - startTime.tv_sec) + endTime1.tv_usec - startTime.tv_usec; 
    printf("timeUse1 = %f\r\n",timeUse1);

    MotorSend.ctrl_motor(id2, 7, 0, 0.5, 0.01, 0);
    gettimeofday(&endTime2,NULL);
    timeUse2 = 1e6*(endTime2.tv_sec - startTime.tv_sec) + endTime2.tv_usec - startTime.tv_usec;
    printf("timeUse2 = %f\r\n",timeUse2);
    //usleep(1e3);
  }
}


void *ctrlReceive(void *data)
{
vector<int> ID;
pdtMotor MotorReceive("can1", ID);
//struct can_frame frame;
//uint16_t id;
//float _pos, _vel, _KP, _KD, _torq;

  while(1)
  MotorReceive.CAN_Receive();
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
