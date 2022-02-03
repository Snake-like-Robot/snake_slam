/**
 * @file snake_pf.cpp
 * @author jeong (lzh_jeong@qq.com)
 * @brief
 * @version 0.1
 * @date 2022-02-02
 *
 * @details
 * @todo 高斯分布归一化常数计算
 */

#include "snake_pf.h"

using namespace snakePF;

PF::PF(pf_coefs coef)
    : coefs(coef)
{
}

PF::~PF() {}

//这里的地图是局部地图
robot_state PF::PfProcess(robot_state last_states, laser_odom::pc laser_scan, Eigen::Matrix2d R, Eigen::Vector2d t, Eigen::MatrixXd grid_map)
{
    /*对t-1时刻的粒子集进行粒子传递*/
    int state_num = last_states.cols();
    robot_state state_trans;
    state_trans.resize(3, state_num);
    state_trans = StateTransfer(R, t, last_states);
    /*对传递后的粒子计算权重(使用观测模型)*/
    weight_list weights;
    weights.resize(1, state_num);
    weights = ObservationModel(state_trans, laser_scan, grid_map);
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

/**
 * @brief
 *
 * @param state_trans
 * @param laser_scan
 * @param grid_map
 * @return weight_list
 * @version 0.1
 * @author jeong (lzh_jeong@qq.com)
 * @date 2022-02-02
 *
 * @details
 * @todo
 */
weight_list PF::ObservationModel(robot_state state_trans, laser_odom::pc laser_scan, Eigen::MatrixXd grid_map)
{
    int state_num = state_trans.cols();
    int beam_num = laser_scan.cols();
    Eigen::Vector2d point_index, point_diff;
    double real_dist, measured_dist;
    weight_list weights;
    weights.resize(1, state_num);
    for (int state_id = 0; state_id < state_num; state_id++)
    {
        // point_index = 给定(x,y)坐标，获取在地图中的id
        for (int beam_id = 0; beam_id < beam_num; beam_id++)
        {
            point_diff = laser_scan.col(beam_id) - state_trans.col(state_id).head(2);
            real_dist = point_diff.norm();
            // point_diff = 从地图中的id得到该栅格中心的位置 - state_trans.col(state_id).head(2);
            measured_dist = point_diff.norm();
            weights(state_id) += BeamRangeFinderModel(real_dist, measured_dist);
        }
    }
    return weights;
}

double PF::BeamRangeFinderModel(double real_dist, double measured_dist)
{
    /*计算高斯分布的概率及其归一化常数*/
    double p_hit_eta = NormalizeFactorCal(real_dist, measured_dist, 10);
    double p_hit = p_hit_eta * GaussianFunctionCal(measured_dist, real_dist);
    /*计算指数分布的概率及其归一化常数*/
    double p_short_eta = 1.0 / (1 - exp(-coefs.lambda_short * real_dist));
    double p_short = p_short_eta * coefs.lambda_short * exp(-coefs.lambda_short * measured_dist);
    /*计算检测失败的情况*/
    double p_max;
    if (measured_dist == coefs.range_max)
        p_max = 1;
    else
        p_max = 0;
    /*计算均匀分布情况下的概率*/
    double p_rand = 1.0 / coefs.range_max;
    /*全部参数综合*/
    double p = coefs.z_hit * p_hit + coefs.z_short * p_short + coefs.z_max * p_max + coefs.z_rand * p_rand;
    return p;
}

double PF::GaussianFunctionCal(double z, double z_star)
{
    return 1.0 / (2 * M_PI * coefs.sigma_hit_frac * z) * exp(-pow(z - z_star, 2) / (2 * pow(coefs.sigma_hit_frac * z, 2)));
}

double PF::NormalizeFactorCal(double real, double measured, int step_num)
{
    double sum = 0, step = coefs.range_max / step_num;
    for (int i = 0; i < step_num; i++)
    {
        sum += 0.5 * (GaussianFunctionCal(step * i, real) + GaussianFunctionCal(step * (i + 1), real)) * step;
    }
    return 1.0 / sum;
}