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
 * @todo 1.地图的详细定义解释
 *       2.粒子滤波器的参数(各种参数)
 *       3.角度的-pi~pi问题
 */
SnakeSlam::SnakeSlam(const std::string &laser_topic_sub, int num)
    : particle_num(num)
{
    std::cout << "----------slam node start!----------" << std::endl;
    _laser_sub = _nh.subscribe(laser_topic_sub, 100, &SnakeSlam::LaserScanCallback, this);

    is_first = true;
    last_state.resize(3, 1);
    last_particles.resize(3, particle_num);
    particles.resize(3, particle_num);
    last_state << 0, 0, 0;
    last_R << 1, 0, 0, 1;
    last_t << 0, 0;

    srand(time(NULL));
    double A = -10.0, B = 10.0;
    for (int i = 0; i < particle_num; i++)
    {
        last_particles(0, i) = A + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (B - A)));
        last_particles(1, i) = A + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (B - A)));
        last_particles(2, i) = 0;
    }

    odom = new laser_odom::LaserOdom(10);
    map = new snake_map::SnakeMap(20, 20, 0.1);

    snakePF::pf_coefs coefs;
    coefs.lambda_short = 1;
    coefs.sigma_hit_frac = 0.03;
    coefs.state_num = particle_num;
    pf = new snakePF::PF(coefs);
}

SnakeSlam::~SnakeSlam()
{
    delete odom;
    delete map;
    delete pf;
}

/**
 * @brief
 *
 * @param scan_msg
 * @version 0.1
 * @author jeong (lzh_jeong@qq.com)
 * @date 2022-02-04
 *
 * @details
 * @todo 1.初始条件下，粒子滤波器不收敛，转换矩阵如何赋值
 */
void SnakeSlam::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    /*第一帧数据需要初始化一个可用地图，坐标系定义为map*/
    if (is_first)
    {
        last_pc = LaserMsg2Pc(scan_msg);
        map->update(last_pc, last_state.topRows(2));
        is_first = false;
        return;
    }
    cur_pc = LaserMsg2Pc(scan_msg);
    /*激光里程计计算，得到两帧之间的位姿变化关系*/
    Eigen::Matrix2d R;
    Eigen::Vector2d t;
    //error 1
    odom->IcpProcess(R, t, cur_pc, last_pc);
    /*局部坐标系下的激光数据点转换到世界坐标系*/
    cur_pc_world = Local2World(last_R * R, last_R * t + last_t, cur_pc);
    /*粒子滤波器*/
    particles = pf->PfProcess(last_particles, cur_pc_world, R, t, map);
    /*基于新的位置更新地图*/
    last_state = 1.0 / particle_num * particles.rowwise().sum();
    map->update(cur_pc_world, last_state.topRows(2));
    /*循环赋值*/
    double theta = last_state(2, 0);
    last_R << cos(theta) , sin(theta),
        sin(theta), cos(theta);
    last_t << last_state(0, 0), last_state(1, 0);
    last_pc = cur_pc;
    last_particles = particles;
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

laser_odom::pc SnakeSlam::Local2World(Eigen::Matrix2d R, Eigen::Vector2d t, laser_odom::pc pc_local)
{
    laser_odom::pc pc_world;
    pc_world.resize(2, pc_local.cols());
    pc_world = R * pc_local + t.replicate(1,pc_local.cols());
    return pc_world;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_test");
    SnakeSlam slam("/course_agv/laser/scan", 50);
    ros::spin();
}