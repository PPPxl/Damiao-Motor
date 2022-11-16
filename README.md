# Damiao-Motor
# Construction of API
## Steps
1. Select a suitable Linux-developed platform. Raspberry Pi is recommended.
2. Set up the C++ developing environment.
3. Set up all the libraries that may be needed, including the Linux CAN library.
4. Code. Integrate all functions into the class. The example can be checked [here](https://github.com/bishopAL/GeRot/tree/master/API/dynamixel_cpp%20Ver2.0).

## Potential problems
1. There could be some differences between *the CAN-HAT* device and *the USB-CAN* device. The code could be different.
2. There may be problems with synchronized reception and transmission of information from multiple motors. Suggested to refer to MIT's open-source solution.
