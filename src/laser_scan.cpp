#include "laser_scan.h"

LaserScan::LaserScan(const std::string &topic)
{
    ROS_INFO_STREAM("LaserScan init!");
    _laser_scan_sub = _node_handle.subscribe(topic, 1000, &LaserScan::LaserScanCallBack, this);
}

LaserScan::~LaserScan()
{
    
}

void LaserScan::LaserScanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan_msgs)
{
    ROS_INFO_STREAM("range max"<<scan_msgs->range_max);
}