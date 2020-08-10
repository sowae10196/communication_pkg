/*
 *
 * communication_main.cpp
 *
 * Created on: 2020. 8. 01
 *     Author: sal9s0
 *       Mail: sowae10196@naver.com
 * 
 */

#include <ros/ros.h>
#include "communication_pkg/communication.hpp"
// #include "communication.cpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "communication_main");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    COMMUNICATION communication_;
    
    while(ros::ok()){
        communication_.checkCommunication();
        // ROS_INFO("test");
        // communication_.dataReceiveTest();
        // communication_.dataTransmitTest();
        communication_.pcuReceive();
        loop_rate.sleep();
    }
    return 0;
}