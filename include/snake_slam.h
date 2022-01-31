#ifndef _SNAKE_SLAM_H_
#define _SNAKE_SLAM_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Dense>

#include "laser_odometer.h"

class SnakeSlam
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber _laser_sub;
    ros::Publisher _map_pub;
    ros::Publisher _location_pub;

public:
    SnakeSlam(const std::string &);
    ~SnakeSlam(){};
    void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &);
};

#endif