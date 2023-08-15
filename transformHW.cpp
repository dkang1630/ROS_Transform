#include <transform_hw/Transformer.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "transformHW");

    ros::NodeHandle nh;
    Transformer transformer(nh);

    ros::spin();
    
    return 0;
}