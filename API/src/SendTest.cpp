#include <iostream>
#include <pdtMotor.h>
using namespace std;

int main( int argc, char** argv )
{

vector<int> ID;
pdtMotor Motor_SendTest("can0", ID);
struct can_frame frame;
uint16_t id = 1;
float _pos, _vel, _KP, _KD, _torq;


for(int i=0; i<=10; i++)
  {
    Motor_SendTest.ctrl_motor(frame, id, _pos, _vel, _KP, _KD, _torq);
  }



// while (1)
// {
//     Motor_SendTest.ctrl_motor(frame, id, _pos, _vel, _KP, _KD, _torq);
// }


 return 0;
}
