#include <lcm/lcm-cpp.hpp>
#include "motionCalculate.h"
#include "robotState/robotState.hpp"
#include "robotCommand/robotCommand.hpp"

lcm::LCM Lcm;
MotionControl motionCtl;


void *thread1_robotMotionCtl(void *data)  //receive motion control command
{
    
}

void *thread2_lcmSendCommand(void *data)  //receive motion control command
{
    
}

void *thread3_lcmRecvData(void *data)  //receive motion control command
{
    
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
    
    param1.sched_priority = PRIORITYRECV;
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
    ret = pthread_create(&thread1, &attr1, thread1_robotMotionCtl, NULL);
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
