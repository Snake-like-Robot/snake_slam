#include "snake_slam.h"

using namespace snake_slam;

/**
 * @brief Construct a new Snake Slam:: Snake Slam object
 *
 * @param laser_topic_sub
 * @version 0.1
 * @author jeong (lzh_jeong@qq.com)
 * @date 2022-02-01
 *
 * @details
 * @todo 1.对last_state进行随机采样的初始化，或者指定位置
 */
SnakeSlam::SnakeSlam(const std::string &laser_topic_sub)
{
    std::cout << "----------slam node start!----------" << std::endl;
    _laser_sub = _nh.subscribe(laser_topic_sub, 100, &SnakeSlam::LaserScanCallback, this);

    is_first = true;
}

void SnakeSlam::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    /*第一帧数据需要初始化一个可用地图，坐标系定义为map*/
    if (is_first)
    {
        last_pc = LaserMsg2Pc(scan_msg);
        is_first = false;
        return;
    }
    cur_pc = LaserMsg2Pc(scan_msg);

    /*激光里程计计算，得到两帧之间的位姿变化关系*/
    Eigen::Matrix2d R;
    Eigen::Vector2d t;
    static laser_odom::LaserOdom odom(10);
    odom.IcpProcess(R, t, cur_pc, last_pc);

    /*基于新的位置更新地图*/
}

/**
 * @brief 返回激光雷达直角坐标系下的数据点列
 *
 * @param scan_msg
 * @return laser_odom::pc
 * @version 0.1
 * @author jeong (lzh_jeong@qq.com)
 * @date 2022-02-01
 *
 * @details
 * @todo
 */
laser_odom::pc SnakeSlam::LaserMsg2Pc(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    laser_odom::pc scan_pc;
    int point_num = scan_msg->ranges.size();
    scan_pc.resize(2, point_num);
    double angle = scan_msg->angle_min;
    for (int i = 0; i < point_num; i++, angle += scan_msg->angle_increment)
    {
        scan_pc(0, i) = scan_msg->ranges.at(i) * cos(angle);
        scan_pc(1, i) = scan_msg->ranges.at(i) * sin(angle);
    }
    return scan_pc;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_test");
    SnakeSlam slam("scan");
    ros::spin();
}