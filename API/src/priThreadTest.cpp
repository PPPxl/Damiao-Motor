#include <iostream>
#include <pdtMotor.h>
#include <math.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "robotCommand/robotCommand.hpp"
#include "handler.h"

using namespace std;

#define PRIORITYRECV 25
#define PRIORITYSEND 30
#define PRIORITYMOTIONCTL 20
#define PRIORITYLCMSEND 10
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
#define THREAD3_ENABLE 1
// #define THREAD4_ENABLE 1


// lcm::LCM Lcm;
// robotCommand::robotCommand rc;
// RobotStateHandler rsHandle;
float ActPos[12] = {0};
float ActVel[12] = {0};
float ActTor[12] = {0};

void *thread1_Recvfunc(void *data)  //receive feedback data from motor
{
    vector<int> ids;
    ids.push_back(1);
    ids.push_back(2);
    pdtMotor motorCtl("can1", ids);

    // ofstream ofs1, ofs2, ofs3;  
    // ofs1.open("pos.txt",ios::out);
    // ofs2.open("vel.txt",ios::out);
    // ofs3.open("tor.txt",ios::out);
    struct timeval startTime,endTime;
    double timeUse;
    while(1)
    {
    // gettimeofday(&startTime,NULL);
    int updatedID = motorCtl.motor_state_receive();
    // gettimeofday(&endTime,NULL);  
    // timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
    //printf("timeUse Recv = %f\r\n",timeUse);
    //cout<<"timeUse Recv: "<<timeUse<<endl;
    ActPos[updatedID-1] = motorCtl.present_position[updatedID-1];
    ActVel[updatedID-1] = motorCtl.present_velocity[updatedID-1];
    ActTor[updatedID-1] = motorCtl.present_torque[updatedID-1];
    // ofs1 <<updatedID<<","<<timeUse<<","<<motorCtl.present_position[updatedID-1]<<endl;
    // ofs2 <<updatedID<<","<<timeUse<<","<<motorCtl.present_velocity[updatedID-1]<<endl;
    // ofs3 <<updatedID<<","<<timeUse<<","<<motorCtl.present_torque[updatedID-1]<<endl;
    }
    // ofs1.close();  
    // ofs2.close();  
    // ofs3.close(); 
}


void *thread2_Sendfunc(void *data) //send commend to motor  
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
    //motor id1 return to zero
    target_pos.push_back(0.0);
    target_vel.push_back(0.0);
    target_KP.push_back(0.5);
    target_KD.push_back(0.01);
    targrt_tor.push_back(0.0);
    //motor id2 return to zero
    target_pos.push_back(0.0);
    target_vel.push_back(0.0);
    target_KP.push_back(0.5);
    target_KD.push_back(0.01);
    targrt_tor.push_back(0.0);
    motorCtl.enable();
    motorCtl.MIT_ctrl_motor(target_pos, target_vel, target_KP, target_KD, targrt_tor);
    usleep(1.5e6);
    float k = 0.0;
    ofstream ofs4;           //ofstream输出文件流
    // ofs4.open("timeuse1.txt",ios::out);
    while(1)
    {
        struct timeval startTime, endTime1;
        double timeUse1;
        gettimeofday(&startTime,NULL);
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
        gettimeofday(&endTime1,NULL);  
        timeUse1 = 1e6*(endTime1.tv_sec - startTime.tv_sec) + endTime1.tv_usec - startTime.tv_usec; 
        // ofs4 <<timeUse1<<endl;   
    }
    // ofs4.close();
}



void *thread3_Motionctl(void *data)  //receive command from motion control level
{
    
}



void *thread4_lcmSendData(void *data)  //send feedback data to motion control level
{
    // robotState::robotState my_robotState;
    // while (1)
    // {
    //     for(int i = 0; i < 12; i++)
    //     {
    //         my_robotState.F[i] = ActTor[i];
    //         my_robotState.endPos[i] = ActPos[i];
    //         my_robotState.endVel[i] = ActVel[i];
    //     }
    //     Lcm.publish("ROBOTSTATE",&my_robotState);
    // }

}







void thread_init()
{
    struct sched_param param1, param2, param3, param4;
    pthread_attr_t attr1, attr2, attr3, attr4;
    pthread_t thread1 ,thread2, thread3, thread4;
    int ret;

    /* 1.Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        printf("mlockall failed: %m\n");
        exit(-2);
    }

    /* 2. Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr1);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
    ret = pthread_attr_init(&attr2);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
    ret = pthread_attr_init(&attr3);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
    ret = pthread_attr_init(&attr4);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }

    /* 3. Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr1, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
    ret = pthread_attr_setstacksize(&attr2, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
    ret = pthread_attr_setstacksize(&attr3, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
    ret = pthread_attr_setstacksize(&attr4, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
 
    /*4. Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr1, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    ret = pthread_attr_setschedpolicy(&attr2, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    ret = pthread_attr_setschedpolicy(&attr3, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    ret = pthread_attr_setschedpolicy(&attr4, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    
    param1.sched_priority = PRIORITYRECV;
    param2.sched_priority = PRIORITYSEND;
    param3.sched_priority = PRIORITYMOTIONCTL;
    param4.sched_priority = PRIORITYLCMSEND;

    ret = pthread_attr_setschedparam(&attr1, &param1);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    ret = pthread_attr_setschedparam(&attr2, &param2);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    ret = pthread_attr_setschedparam(&attr3, &param3);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    ret = pthread_attr_setschedparam(&attr4, &param4);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }

    /*5. Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr1, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
    ret = pthread_attr_setinheritsched(&attr2, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
    ret = pthread_attr_setinheritsched(&attr3, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
    ret = pthread_attr_setinheritsched(&attr4, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
 
    /*6. Create a pthread with specified attributes */
    #ifdef THREAD1_ENABLE
    ret = pthread_create(&thread1, &attr1, thread1_Recvfunc, NULL);
    if (ret) {
            printf("create pthread1 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD2_ENABLE
    ret = pthread_create(&thread2, &attr2, thread2_Sendfunc, NULL);
    if (ret) {
            printf("create pthread2 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD3_ENABLE
    ret = pthread_create(&thread3, &attr3, thread3_Motionctl, NULL);
    if (ret) {
            printf("create pthread3 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD4_ENABLE
    ret = pthread_create(&thread4, &attr4, thread4_lcmSendData, NULL);
    if (ret) {
            printf("create pthread4 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD1_ENABLE
    ret = pthread_join(thread1, NULL);
    if (ret)
        printf("join pthread1 failed: %m\n");
    #endif

    #ifdef THREAD2_ENABLE
    ret = pthread_join(thread2, NULL);
    if (ret)
        printf("join pthread2 failed: %m\n");
    #endif
 
    #ifdef THREAD3_ENABLE
    ret = pthread_join(thread3, NULL);
    if (ret)
        printf("join pthread3 failed: %m\n");
    #endif

    #ifdef THREAD4_ENABLE
    ret = pthread_join(thread4, NULL);
    if (ret)
        printf("join pthread4 failed: %m\n");
    #endif

    /*7. Join the thread and wait until it is done */   
out:
    ret;
}


int main( int argc, char** argv )
{
thread_init();
return 0;
}
