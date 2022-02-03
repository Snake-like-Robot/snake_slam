#ifndef _SNAKE_PF_H_
#define _SNAKE_PF_H_

#include <eigen3/Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>

#include "laser_odometer.h"

namespace snakePF
{
    typedef Eigen::Matrix<double, 3, -1> robot_state; // x,y,theta
    typedef Eigen::Matrix<double, 1, -1> weight_list;

    struct pf_coefs
    {
        double sigma_hit;
        double sigma_hit_frac; //或者用当前激光数据点的距离大小的比例表示方差
        double range_max;
        double lambda_short;
        double z_hit;
        double z_short;
        double z_max;
        double z_rand;
    };

    class PF
    {
    private:
        pf_coefs coefs;

    public:
        PF(pf_coefs);
        ~PF();
        robot_state PfProcess(robot_state, laser_odom::pc, Eigen::Matrix2d, Eigen::Vector2d, Eigen::MatrixXd);
        robot_state StateTransfer(Eigen::Matrix2d, Eigen::Vector2d, robot_state);
        weight_list ObservationModel(robot_state, laser_odom::pc, Eigen::MatrixXd);
        double BeamRangeFinderModel(double, double);
        double NormalizeFactorCal(double, double, int);
        double GaussianFunctionCal(double,double);
    };
}

#endif