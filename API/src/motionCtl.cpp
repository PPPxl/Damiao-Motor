#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "robotCommand/robotCommand.hpp"
#include "handler.h"
#include "motionCalculate.h"
#include <pdtMotor.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>

using namespace std;

#define PRIORITYLCMRECV 10
#define PRIORITYSEND 10
#define PRIORITYUPDATE 5
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
#define THREAD3_ENABLE 1
//  1:  Motor angle
//  2:  Foot end position
#define INIMODE 2
#define _JOYSTICK 1
#define MORTOR_ANGLE_AMP 25*3.14/180.0
#define loopRateCommandUpdate 100.0   //hz
#define loopRateStateUpdateSend 20.0   //hz
#define loopRateImpCtller 100.0   //hz
#define VELX 8.0 /1000   // mm  step length = VELX * timeForStancePhase        
#define TimePeriod 0.05
#define TimeForGaitPeriod 6

lcm::LCM Lcm;
RobotStateHandler rsHandle;

// vector<int> ID = {  
// 1,2,3, 
// 4,5,6,
// 7,8,9,
// 10,11,12
// };

// pdtMotor motors("can1", ID);
// vector<float> SetPos(12), SetVel(12), SetTorque(12);


void *thread1_robotCommandUpdate(void *data)  //receive motion control command
{
   /* Matrix<float, 4, 2>TimeForStancePhase;
    Matrix<float, 4, 3> InitPos;
    Vector<float, 3> TCV={ VELX, 0, 0 };// X, Y , alpha 

    //motors initial
    motors.setOperatingMode(3);  //3 position control; 0 current control
    motors.torqueEnable();
    motors.getPosition();
    usleep(1e6);
#if(INIMODE==1)
    vector<float> init_Motor_angle(12);
    float float_init_Motor_angle[12];
    string2float("../include/init_Motor_angle.csv", float_init_Motor_angle);//Motor angle     d
    //cout<<"____________"<<endl;
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            float_init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j] * 3.1416/180; //to rad
            init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j];      //vector
            imp.jointCmdPos(i,j) = float_init_Motor_angle[i*3+j];            //imp.forwardKinematics
            //cout<<init_Motor_angle[i*3+j]<<endl;
        }
    imp.forwardKinematics(0);
    imp.setInitPos(imp.legCmdPos);        //legCmdPos
    cout<<"legCmdPos:\n"<<imp.legCmdPos<<endl ;

    motors.setPosition(init_Motor_angle);
#endif 

  //      imp initial
    // TimeForStancePhase<< 0,                       TimeForGaitPeriod/2.0,     // diagonal
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      0,                       TimeForGaitPeriod/2.0;
    TimeForStancePhase<< TimeForGaitPeriod/4.0 *3,          TimeForGaitPeriod/4.0 *2,   // tripod
                         TimeForGaitPeriod/4.0,             TimeForGaitPeriod,
                         TimeForGaitPeriod - TimePeriod,    TimeForGaitPeriod/4.0 *3,
                         TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0;
    imp.setPhase(TimePeriod, TimeForGaitPeriod, TimeForStancePhase);

#if(INIMODE==2)
    float  float_initPos[12];
    string2float("../include/initPos.csv", float_initPos);//Foot end position
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            InitPos(i, j) = float_initPos[i*3+j];
            //cout<<InitPos(i, j)<<endl;
        }
    imp.setInitPos(InitPos);
#endif 

 imp.setCoMVel(TCV);
    imp.inverseKinematics(imp.legCmdPos);
#if(INIMODE==2)  
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
           
            SetPos[i*3+j] = imp.jointCmdPos(i,j);
            cout<<"SetPos:"<<SetPos[i*3+j] <<endl;
            if( isnanf(SetPos[i*3+j]) )
                SetPos[i*3+j] = 0;
        }
    motors.setPosition(SetPos);     
#endif

imp.target_pos = imp.legCmdPos;
    imp.initFlag = 1;
    usleep(1e5);
    while(1)
    {
        struct timeval startTime,endTime;
        double timeUse;
        gettimeofday(&startTime,NULL);

        //If stay static, annotate below one line.
        imp.nextStep();//
        imp.impParaDeliver();
        // cout<<"legCmdPos:\n"<<imp.legCmdPos<<endl;

        gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        if(timeUse < 1e4)
            usleep(1.0/loopRateStateUpdateSend*1e6 - (double)(timeUse) - 10); 
        else
            cout<<"TimeRobotStateUpdateSend: "<<timeUse<<endl;

    }*/

}

void *thread2_lcmSendCommand(void *data)  //receive motion control command
{
   

}

void *thread3_lcmRecvData(void *data)  //receive robot state data from joint control level
{
    
    Lcm.subscribe("ROBOTSTATE", &RobotStateHandler::handleMessage, &rsHandle);
   
    while(0 == Lcm.handle());
}



void motionCtlthread_init()
{
    struct sched_param param1, param2, param3;
    pthread_attr_t attr1, attr2, attr3;
    pthread_t thread1 ,thread2, thread3;
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
    
    param1.sched_priority = PRIORITYUPDATE;
    param2.sched_priority = PRIORITYSEND;
    param3.sched_priority = PRIORITYLCMRECV;
    

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
 
    /*6. Create a pthread with specified attributes */
    #ifdef THREAD1_ENABLE
    ret = pthread_create(&thread1, &attr1, thread1_robotCommandUpdate, NULL);
    if (ret) {
            printf("create pthread1 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD2_ENABLE
    ret = pthread_create(&thread2, &attr2, thread2_lcmSendCommand, NULL);
    if (ret) {
            printf("create pthread2 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD3_ENABLE
    ret = pthread_create(&thread3, &attr3, thread3_lcmRecvData, NULL);
    if (ret) {
            printf("create pthread3 failed\n");
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

    /*7. Join the thread and wait until it is done */   
out:
    ret;
}



int main( int argc, char** argv )
{
motionCtlthread_init();
return 0;
}
