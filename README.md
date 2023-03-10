# Damiao-Motor

# Explanation of API
![image](https://user-images.githubusercontent.com/112753161/224244681-fb7f106e-d253-4d48-8542-35509c988153.png)
1. The pdtMotor.cpp is the API of the motor, the Raspberry Pi with CAN bus communication is needed with this version.
2. The motionCalculate.cpp and motionCtl.cpp are the motion control level of the climbing robot, this section is not completely completed.
3. The oneThreadTest.cpp, priThreadTest.cpp and twoThreadTest.cpp are the tests of Multi-Threads, just ignored it.
4. The handler.cpp is used for the LCM.
5. The testRecv.cpp is used for the receive the feedback.

# Construction of API
## Steps
1. Select a suitable Linux-developed platform. Raspberry Pi is recommended.
2. Set up the C++ developing environment.
3. Set up all the libraries that may be needed, including the Linux CAN library.
4. Code. Integrate all functions into the class. The example can be checked [here](https://github.com/bishopAL/GeRot/tree/master/API/dynamixel_cpp%20Ver2.0).

## Potential problems
1. There could be some differences between *the CAN-HAT* device and *the USB-CAN* device. The code could be different.
2. There may be problems with synchronized reception and transmission of information from multiple motors. Suggested to refer to MIT's open-source solution.
