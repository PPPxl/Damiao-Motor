#include <iostream>
#include <pdtMotor.h>
#include <unistd.h>
using namespace std;

int main( int argc, char** argv )
{

vector<int> ID;
pdtMotor Motor_SendTest("can1", ID);
struct can_frame frame;
uint16_t id2 = 2;
uint16_t id1 = 1;

Motor_SendTest.enable(id2);
Motor_SendTest.enable(id1);
for(int i=0; i<=10; i++)
  {
    Motor_SendTest.ctrl_motor(id2, 6, 0, 0.5, 0.01, 0);

    Motor_SendTest.ctrl_motor(id1, 6, 0, 0.5, 0.01, 0);

    usleep(1e3);
  }



// while (1)
// {
//     Motor_SendTest.ctrl_motor(frame, id, _pos, _vel, _KP, _KD, _torq);
// }


 return 0;
}
