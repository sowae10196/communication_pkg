#include <ros/ros.h>

void testFunction(uint8_t *array_){
    array_[0] += 1;
    array_[1] += 2;
    array_[2] += 3;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    
    uint8_t array[3];
    
    memset(array, 0, 3);

    while(ros::ok()){
        testFunction(array);
        for(int i = 0; i < 3; i++)
            printf("%d ", array[i]);
        printf("\n");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}