/*
 *
 * communication.hpp
 *
 * Created on: 2020. 7. 23
 *      Modifier: sal9s0
 *      Mail: sowae10196@naver.com
 * 
 */

#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include <ros/ros.h>
#include <serial/serial.h>

#define READ_BUFF_SIZE 18
#define WRITE_BUFF_SIZE 14

// Write Register Address, Big Endian
#define WRITE_S         0
#define WRITE_T         1
#define WRITE_X         2
#define WRITE_AorM      3
#define WRITE_E_STOP    4
#define WRITE_GEAR      5
#define WRITE_SPEED0    6
#define WRITE_SPEED1    7
#define WRITE_STEER0    8
#define WRITE_STEER1    9
#define WRITE_BRAKE     10
#define WRITE_ALIVE     11
#define WRITE_ETX0      12
#define WRITE_ETX1      13

// Read Register Address, Little Endian
#define READ_S          0
#define READ_T          1
#define READ_X          2
#define READ_AorM       3   
#define READ_E_STOP     4
#define READ_GEAR       5
#define READ_SPEED0     6
#define READ_SPEED1     7   
#define READ_STEER0     8
#define READ_STEER1     9
#define READ_BRAKE      10
#define READ_ENC0       11
#define READ_ENC1       12
#define READ_ENC2       13
#define READ_ENC3       14
#define READ_ALIVE      15
#define READ_ETX0       16
#define READ_ETX1       17

// Control Value
#define MANUAL          0x00
#define AUTO            0x01
#define E_STOP_OFF      0x00
#define E_STOP_ON       0x01
#define FORWARD_DRIVE   0x00
#define NEUTRAL         0x01
#define BACKWARD_DRIVE  0x02
#define NO_BRAKING      1
#define FULL_BRAKING    200

// Write Value
#define FORWARD true
#define BACKWARD false

class COMMUNICATION
{
private:
    // ROS settings
    ros::NodeHandle nh;

    // serial settings
    serial::Serial serial_;

    // param settings
    std::string paramPortName;
    int paramBaudRate;

    // buffer
    uint8_t readBuffer[READ_BUFF_SIZE];
    uint8_t writeBuffer[WRITE_BUFF_SIZE];

    // read value
    int current_speed;
    int current_steer;
    int current_brake;
    int current_enc;
    uint8_t write_alive;


    // Temporal Var: After Test, delete it
    bool m_bIsFirstConnected = true;
	bool m_bIsFirstDisconnected = true;
	bool m_bConnectResult = false;
    void setParam();
    void init();

public:
    COMMUNICATION();
    ~COMMUNICATION();

    void startCommunication();
    void checkCommunication();
    void setWriteData(int goalSpeed, int goalSteer, bool gear);
    void pcuTransmit();
    void pcuReceive();

    // test Functions
    void dataTransmitTest();
    void dataReceiveTest();
};

#endif //__COMMUNICATION_H__