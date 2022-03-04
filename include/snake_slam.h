#ifndef _SNAKE_SLAM_H_
#define _SNAKE_SLAM_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <ctime>
#include <cstdlib>

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
        ros::Publisher _marker_pub;
        visualization_msgs::Marker points;
        tf::TransformBroadcaster _tf_broadcaster;
        laser_odom::pc last_pc;
        laser_odom::pc cur_pc;
        laser_odom::pc cur_pc_world;
        bool is_first;
        snakePF::robot_state last_state;
        snakePF::robot_state last_particles;
        snakePF::robot_state particles;
        Eigen::Matrix2d last_R;
        Eigen::Vector2d last_t;
        int particle_num;
        int abort_num;
        laser_odom::LaserOdom *odom;
        snake_map::SnakeMap *map;
        snakePF::PF *pf;

    public:
        SnakeSlam(const std::string &, const std::string &, int);
        ~SnakeSlam();
        void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &);
        laser_odom::pc LaserMsg2Pc(const sensor_msgs::LaserScan::ConstPtr &);
        laser_odom::pc Local2World(Eigen::Matrix2d, Eigen::Vector2d, laser_odom::pc);
        void MarkerVisualize(laser_odom::pc);
        void MarkerVisualize(snakePF::robot_state);
        double RandNum(double, double);
        void Test();
    };
};
#endif