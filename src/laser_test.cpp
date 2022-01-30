#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include "laser_scan.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_test");
    LaserScan laser_scan("scan");
    ros::spin();
}