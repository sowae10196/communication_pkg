/*
 *
 * communication.cpp
 *
 * Created on: 2020. 7. 23
 *      Modifier: sal9s0
 *      Mail: sowae10196@naver.com
 * 
 */

#include <ros/ros.h>
#include "communication_pkg/communication.hpp"

COMMUNICATION::COMMUNICATION(){
    init();

    startCommunication();
    checkCommunication();
}

COMMUNICATION::~COMMUNICATION(){
}

void COMMUNICATION::setParam(){
    nh.param<std::string>("communication/paramPortName", paramPortName, "/dev/ttyUSB0");
    nh.param<int>("communication/paramBaudRate", paramBaudRate, 115200);
}

void COMMUNICATION::init(){
    // set parameters
    setParam();
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

    // serial initialize
    serial_.setPort(paramPortName);
    serial_.setBaudrate(paramBaudRate);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial_.setFlowcontrol(serial::flowcontrol_hardware);
    serial_.setTimeout(timeout);

    // buffer initialize
    memset(readBuffer, 0x00, READ_BUFF_SIZE);
    memset(writeBuffer, 0x00, WRITE_BUFF_SIZE);

    writeBuffer[WRITE_S] = (uint8_t)0x53;
    writeBuffer[WRITE_T] = (uint8_t)0x54;
    writeBuffer[WRITE_X] = (uint8_t)0x58;
    writeBuffer[WRITE_AorM] = (uint8_t)AUTO;
    writeBuffer[WRITE_E_STOP] = (uint8_t)E_STOP_OFF;
    writeBuffer[WRITE_GEAR] = (uint8_t)NEUTRAL;
    writeBuffer[WRITE_SPEED0] = (uint8_t)0x00;
    writeBuffer[WRITE_SPEED1] = (uint8_t)0x00;
    writeBuffer[WRITE_STEER0] = (uint8_t)0x00;
    writeBuffer[WRITE_STEER1] = (uint8_t)0x00;
    writeBuffer[WRITE_BRAKE] = (uint8_t)100;
    writeBuffer[WRITE_ALIVE] = (uint8_t)0x00;
    writeBuffer[WRITE_ETX0] = (uint8_t)0x0D;
    writeBuffer[WRITE_ETX1] = (uint8_t)0x0A;
}

void COMMUNICATION::startCommunication(){
    try	{
		ROS_INFO_STREAM("Request port open...");
		serial_.open();
	}
	catch (serial::IOException& e)  {
        ROS_ERROR_STREAM("Unable to open port ");
    }
}

void COMMUNICATION::checkCommunication(){       // After test, change it
	if(serial_.isOpen()){
		if (m_bIsFirstConnected)
		{
			m_bIsFirstConnected = false;
			m_bIsFirstDisconnected = true;
			ROS_INFO_STREAM("Serial Port connected");

			m_bConnectResult = true;
		}
	}else{
		if (m_bIsFirstDisconnected)
		{
			m_bIsFirstDisconnected = false;
			m_bIsFirstConnected = true;
			ROS_INFO_STREAM("Serial Port disconnected");

			m_bConnectResult = false;
		}
	}
}

void COMMUNICATION::setWriteData(int goalSpeed, int goalSteer, bool gear){
    uint8_t speed0, speed1, steer0, steer1, brake;




    memset(writeBuffer, 0x00, WRITE_BUFF_SIZE);

    writeBuffer[WRITE_S] = (uint8_t)0x53;
    writeBuffer[WRITE_T] = (uint8_t)0x54;
    writeBuffer[WRITE_X] = (uint8_t)0x58;
    writeBuffer[WRITE_AorM] = (uint8_t)AUTO;
    writeBuffer[WRITE_ALIVE] = (uint8_t)write_alive;     // Set After Receive Data
    writeBuffer[WRITE_ETX0] = (uint8_t)0x0D;
    writeBuffer[WRITE_ETX1] = (uint8_t)0x0A;

    writeBuffer[WRITE_SPEED0] = (uint8_t)0x00;
    writeBuffer[WRITE_SPEED1] = (uint8_t)0x00;
    writeBuffer[WRITE_STEER0] = (uint8_t)0x00;
    writeBuffer[WRITE_STEER1] = (uint8_t)0x00;
    writeBuffer[WRITE_GEAR] = (uint8_t)FORWARD_DRIVE;
    writeBuffer[WRITE_E_STOP] = (uint8_t)E_STOP_OFF;
    writeBuffer[WRITE_BRAKE] = (uint8_t)NO_BRAKING;
}


void COMMUNICATION::pcuTransmit(){
    // data transmit
    ROS_INFO_STREAM("Writing to serial port");

    serial_.write(writeBuffer, WRITE_BUFF_SIZE);
}

void COMMUNICATION::pcuReceive(){
    // data receive
    ROS_INFO_STREAM("Reading from serial port");
    memset(readBuffer, 0x00, READ_BUFF_SIZE);
    serial_.read(readBuffer, serial_.available());

    // data parsing

    // current_speed = 
    // current_steer = 
    // current_brake = 
    // current_enc = 

    // Need Test
    
    // writeBuffer[WRITE_ALIVE] = readBuffer[WRITE_ALIVE];
    write_alive = readBuffer[WRITE_ALIVE];
}

void COMMUNICATION::dataTransmitTest(){
    static uint8_t steer_test = 0x00;
    memset(writeBuffer, 0x00, WRITE_BUFF_SIZE);
    writeBuffer[WRITE_S] = (uint8_t)0x53;
    writeBuffer[WRITE_T] = (uint8_t)0x54;
    writeBuffer[WRITE_X] = (uint8_t)0x58;
    writeBuffer[WRITE_AorM] = (uint8_t)AUTO;
    writeBuffer[WRITE_E_STOP] = (uint8_t)E_STOP_OFF;
    writeBuffer[WRITE_GEAR] = (uint8_t)FORWARD_DRIVE;
    writeBuffer[WRITE_SPEED0] = (uint8_t)0x00;
    writeBuffer[WRITE_SPEED1] = (uint8_t)0x00;
    writeBuffer[WRITE_STEER0] = (uint8_t)0x00;
    writeBuffer[WRITE_STEER1] = (uint8_t)steer_test;
    writeBuffer[WRITE_BRAKE] = (uint8_t)NO_BRAKING;
    writeBuffer[WRITE_ALIVE] = (uint8_t)write_alive;
    writeBuffer[WRITE_ETX0] = (uint8_t)0x0D;
    writeBuffer[WRITE_ETX1] = (uint8_t)0x0A;

    steer_test++;
    if(steer_test >= 0x9F)
        steer_test = 0x00;

    // serial_.write(writeBuffer, WRITE_BUFF_SIZE);

    for(int i = 0; i < WRITE_BUFF_SIZE; i++)
        std::cout << (int)writeBuffer[i] << ", ";
    std::cout << std::endl;

    serial_.write(writeBuffer, WRITE_BUFF_SIZE);
}

void COMMUNICATION::dataReceiveTest(){
    memset(readBuffer, 0x00, READ_BUFF_SIZE);
    serial_.read(readBuffer, serial_.available());

    write_alive = readBuffer[15];

    for(int i = 0; i < READ_BUFF_SIZE; i++)
        std::cout << (int)readBuffer[i] << ", ";
    std::cout << std::endl;
}

