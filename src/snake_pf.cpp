#include "snake_pf.h"

using namespace snakePF;

PF::PF() {}

PF::~PF() {}

robot_state PF::PfProcess(robot_state last_states, Eigen::Matrix2d R, Eigen::Vector2d t)
{
    /*对t-1时刻的粒子集进行粒子传递*/
    int state_num = last_states.cols();
    robot_state state_trans;
    state_trans.resize(3, state_num);
    state_trans = StateTransfer(R, t, last_states);
    /*对传递后的粒子计算权重(使用观测模型)*/

    /*基于重要性的重采样，得到新的粒子集*/
    return state_trans;
}

robot_state PF::StateTransfer(Eigen::Matrix2d R, Eigen::Vector2d t, robot_state last_state)
{
    robot_state state_trans;
    int state_num = last_state.cols();
    state_trans.resize(3, state_num);
    double dtheta = atan2(R(1, 0), R(0, 0));
    double delta_rot1, delta_trans = t.norm();
    for (int i = 0; i < state_num; i++)
    {
        delta_rot1 = atan2(t(1, 0), t(0, 0)) - last_state(2, i);
        state_trans(0, i) = last_state(0, i) + delta_trans * cos(delta_rot1 + last_state(2, i));
        state_trans(1, i) = last_state(1, i) + delta_trans * sin(delta_rot1 + last_state(2, i));
        state_trans(2, i) = last_state(2, i) + dtheta;
    }
    return state_trans;
}
