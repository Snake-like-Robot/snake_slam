#include "snake_slam.h"

using namespace snake_slam;

#define isTest false

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
SnakeSlam::SnakeSlam(const std::string &laser_topic_sub, const std::string &map_topic_pub, int num)
    : particle_num(num)
{
    std::cout << "----------slam node start!----------" << std::endl;
#if !isTest
    _laser_sub = _nh.subscribe(laser_topic_sub, 100, &SnakeSlam::LaserScanCallback, this);
#endif
    _map_pub = _nh.advertise<nav_msgs::OccupancyGrid>(map_topic_pub, 100);
    _marker_pub = _nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

    is_first = true;
    last_state.resize(3, 1);
    last_particles.resize(3, particle_num);
    particles.resize(3, particle_num);
    last_state << 0, 0, 0;
    last_R << 1, 0, 0, 1;
    last_t << 0, 0;

    abort_num = 0;

    srand((unsigned)time(NULL));
    double lenx = 129, leny = 129, xyreso = 0.155;
    for (int i = 0; i < particle_num; i++)
    {
        last_particles(0, i) = RandNum(-0.5 * lenx * xyreso, 0.5 * lenx * xyreso);
        last_particles(1, i) = RandNum(-0.5 * leny * xyreso, 0.5 * leny * xyreso);
        last_particles(2, i) = 0;
    }

    MarkerVisualize(last_particles);

    odom = new laser_odom::LaserOdom(30);
    map = new snake_map::SnakeMap(lenx, leny, xyreso);

    snakePF::pf_coefs coefs;
    coefs.lambda_short = 1;
    coefs.sigma_hit_frac = 0.03;
    coefs.state_num = particle_num;
    coefs.z_hit = 0.25;
    coefs.z_max = 0.25;
    coefs.z_rand = 0.25;
    coefs.z_short = 0.25;
    pf = new snakePF::PF(coefs);

    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.1;
    points.scale.y = 0.1;

    points.color.r = 1.0f;
    points.color.a = 1.0f;

    std::cout << "init finished!" << std::endl;

#if isTest
    Test();
#endif
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
    ROS_INFO_STREAM("msg receive!");
    if (abort_num < 5)
    {
        abort_num++;
        return;
    }
    /*第一帧数据需要初始化一个可用地图，坐标系定义为map*/
    if (is_first)
    {
        last_pc = LaserMsg2Pc(scan_msg);
        map->update(last_pc, Eigen::Vector2d::Zero());
        ROS_INFO_STREAM("map update!");

        snakePF::pf_coefs coefs;
        coefs = pf->CoefGet();
        coefs.range_max = scan_msg->range_max;
        pf->CoefUpdate(coefs);

        is_first = false;
        return;
    }
    cur_pc = LaserMsg2Pc(scan_msg);

    /*激光里程计计算，得到两帧之间的位姿变化关系*/
    Eigen::Matrix2d R;
    Eigen::Vector2d t;
    odom->IcpProcess(R, t, cur_pc, last_pc);

    last_R = last_R * R;
    last_t = last_R * t + last_t;

    laser_odom::pc cur_pc_world = Local2World(last_R, last_t, cur_pc);

    last_particles = pf->PfProcess(last_particles, cur_pc_world, R, t, map);

    Eigen::Vector2d last_center = last_particles.topRows(2).rowwise().sum();

    std::cout << "--------center--------" << std::endl
              << last_center << std::endl;

    map->update(cur_pc_world, last_center);

    MarkerVisualize(last_particles);

    last_pc = cur_pc;

    _map_pub.publish(map->rviz_map);
    ROS_INFO_STREAM("Done");
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
    // pc_world = R.transpose() * (pc_local - t.replicate(1, pc_local.cols()));
    pc_world = R * pc_local + t.replicate(1, pc_local.cols());
    return pc_world;
}

void SnakeSlam::MarkerVisualize(laser_odom::pc point_cloud)
{

    geometry_msgs::Point p;

    int pc_num = point_cloud.cols();

    points.points.clear();

    for (int i = 0; i < pc_num; i++)
    {
        p.x = point_cloud(0, i);
        p.y = point_cloud(1, i);
        p.z = 0;
        points.points.push_back(p);
    }

    _marker_pub.publish(points);
    ROS_INFO_STREAM("Markers!");
}

void SnakeSlam::MarkerVisualize(snakePF::robot_state particles)
{
    MarkerVisualize((laser_odom::pc)particles.topRows(2));
}

double SnakeSlam::RandNum(double low, double high)
{
    return low + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (high - low)));
}

void SnakeSlam::Test()
{
    ros::Rate loop(10);

    laser_odom::pc test_markers;

    while (ros::ok())
    {
        std::cout << last_particles << std::endl;
        MarkerVisualize(last_particles);
        ros::spinOnce();
        loop.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snake_slam_node");
    SnakeSlam slam("/course_agv/laser/scan", "/srtp/map", 20);
    ros::spin();
}