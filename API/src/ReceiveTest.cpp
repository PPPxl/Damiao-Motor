#include <iostream>
#include <pdtMotor.h>
using namespace std;

int main( int argc, char** argv )
{

vector<int> ID;
pdtMotor Motor_ReceiveTest("can1", ID);
//struct can_frame frame;
//uint16_t id;
//float _pos, _vel, _KP, _KD, _torq;

for(int i=0; i<=4; i++)
  {
    Motor_ReceiveTest.CAN_Receive();
  }

 return 0;
}
