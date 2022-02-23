#ifndef _LASER_ODOMETER_H_
#define _LASER_ODOMETER_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

namespace laser_odom
{
    typedef Eigen::Matrix<double, 2, -1> pc;
    typedef Eigen::Matrix<double, 1, -1> row_vector;

    class LaserOdom
    {
    private:
        int max_iter_cnt;
        double last_score;
        double icp_err_change_threshold = 1e-6;

    public:
        LaserOdom(int);
        ~LaserOdom(){};
        void IcpProcess(Eigen::Matrix2d &, Eigen::Vector2d &, pc, pc);
        pc ClosestMatch(pc, pc);
        double MatchScoreEvaluate(Eigen::Matrix2d, Eigen::Vector2d, pc, pc);
    };
}
#endif