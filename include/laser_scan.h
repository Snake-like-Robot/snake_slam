#ifndef _LASER_SCAN_H_
#define _LASER_SCAN_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserScan
{
    private:
        ros::NodeHandle _node_handle;
        ros::Subscriber _laser_scan_sub;
    public:
        LaserScan(const std::string &topic);
        ~LaserScan();
        void LaserScanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan_msgs);
};

#endif