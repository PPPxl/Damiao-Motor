#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
class Handler 
{
    public:
        ~Handler() {}
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const robotState::robotState* msg)
        {
            int i;
            printf("  F   = %f, %f\n", msg->F[0], msg->F[1] );
            printf("  position    = (%f, %f)\n",
                    msg->endPos[0], msg->endPos[1]);
            printf("  endVel = (%f, %f)\n",
                    msg->endVel[0], msg->endVel[1]);

        }
};

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    Handler handlerObject;
    lcm.subscribe("ROBOTSTATE", &Handler::handleMessage, &handlerObject);
    while(0 == lcm.handle());
    return 0;
}