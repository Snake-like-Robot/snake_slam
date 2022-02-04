#ifndef _SNAKE_SLAM_H_
#define _SNAKE_SLAM_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Dense>

#include "laser_odometer.h"
#include "mapping.h"
#include "snake_pf.h"

namespace snake_slam
{
    class SnakeSlam
    {
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _laser_sub;
        ros::Publisher _map_pub;
        ros::Publisher _location_pub;
        laser_odom::pc last_pc;
        laser_odom::pc cur_pc;
        bool is_first;

    public:
        SnakeSlam(const std::string &);
        ~SnakeSlam();
        void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &);
        laser_odom::pc LaserMsg2Pc(const sensor_msgs::LaserScan::ConstPtr &);
    };
};
#endif