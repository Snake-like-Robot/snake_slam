#ifndef _PARTICLE_FILTERS_H_
#define _PARTICLE_FILTERS_H_

#include <eigen3/Eigen/Dense>

namespace particlefilter
{
    typedef Eigen::Matrix<double, 3, -1> robot_state; // x,y,theta
    class PF
    {
    private:
    public:
        PF(){};
        ~PF(){};
        robot_state PfProcess(robot_state, Eigen::Matrix2d, Eigen::Vector2d);
        robot_state StateTransfer(Eigen::Matrix2d, Eigen::Vector2d, robot_state);
    };
};

#endif