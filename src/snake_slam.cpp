#include "snake_slam.h"

SnakeSlam::SnakeSlam(const std::string &laser_topic_sub)
{
    std::cout << "----------slam node start!----------" << std::endl;
    _laser_sub = _nh.subscribe(laser_topic_sub, 100, &SnakeSlam::LaserScanCallback, this);
}

void SnakeSlam::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{

    /*激光里程计计算，得到两帧之间的位姿变化关系*/

    /*对t-1时刻的粒子集进行状态转移*/

    /*对转移后的粒子计算权重(使用观测模型)*/

    /*基于重要性的重采样，得到新的粒子集*/

    /*基于新的位置更新地图*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_test");
    SnakeSlam slam("scan");
    ros::spin();
}