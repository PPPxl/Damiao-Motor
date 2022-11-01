#include <iostream>
#include <pdtMotor.h>
#include <unistd.h>
using namespace std;

int main( int argc, char** argv )
{

vector<int> ID;
pdtMotor Motor_SendTest("can0", ID);
struct can_frame frame;
uint16_t id = 2;
float _pos, _vel, _KP, _KD, _torq;

Motor_SendTest.enable(id);
for(int i=0; i<=10; i++)
  {
    Motor_SendTest.ctrl_motor(id, 2, 0, 0.5, 0.01, 0);
    usleep(1e3);
  }



// while (1)
// {
//     Motor_SendTest.ctrl_motor(frame, id, _pos, _vel, _KP, _KD, _torq);
// }


 return 0;
}
