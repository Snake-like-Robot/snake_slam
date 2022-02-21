#include "laser_odometer.h"

using namespace laser_odom;

LaserOdom::LaserOdom(int max_iter) : max_iter_cnt(max_iter)
{
    last_score = 1000000;
}

/**
 * @brief point-to-point icp
 *
 * @param rot_mat
 * @param trans_vector
 * @param src
 * @param tar
 * @version 0.1
 * @author jeong (lzh_jeong@qq.com)
 * @date 2022-02-01
 *
 * @details 1.求解旋转矩阵R和平移向量t，将source_pc(P')(后一帧)变换到target_pc(P)(前一帧)
 *          2.不需要将激光雷达的数据预先转换到世界坐标系
 *          3.point cloud矩阵的维度为2*n
 * @todo
 */
void LaserOdom::IcpProcess(Eigen::Matrix2d &rot_mat, Eigen::Vector2d &trans_vector, pc src, pc tar)
{
    int src_point_num = src.cols();

    pc tar_matched, src_decenter, tar_decenter;
    Eigen::Vector2d src_center, tar_center;
    Eigen::Matrix2d W, U, V, R;
    Eigen::Vector2d t;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double score;
    for (int iter_cnt = 0; iter_cnt < max_iter_cnt; iter_cnt++)
    {
        // std::cout << "ICP iteration count:" << iter_cnt << std::endl; //用于调试误差变化阈值
        tar_matched = ClosestMatch(src, tar);
        /*计算去质心坐标*/
        src_center = 1.0 / src_point_num * src.rowwise().sum();              // p'
        tar_center = 1.0 / src_point_num * tar_matched.rowwise().sum();      // p
        src_decenter = src - src_center.replicate(1, src_point_num);         // q'
        tar_decenter = tar_matched - tar_center.replicate(1, src_point_num); // q
        /*SVD分解计算旋转矩阵*/
        W = src_decenter * tar_decenter.transpose();
        V = svd.matrixV();
        U = svd.matrixU();
        R = V * U.transpose();
        t = tar_center - R * src_center;
        /*对结果进行评估*/
        score = MatchScoreEvaluate(R, t, src, tar_matched);
        /*实施变换*/
        // src = R * src + t.replicate(1, src_point_num);

        if (score < last_score && last_score - score < icp_err_change_threshold)
        {
            break;
        }
        last_score = score;
    }
    /*返回旋转矩阵与平移向量*/
    rot_mat = R;
    trans_vector = t;
}

/**
 * @brief ICP算法中的最近点匹配
 *
 * @param src
 * @param tar
 * @return pc
 * @version 0.1
 * @author jeong (lzh_jeong@qq.com)
 * @date 2022-01-31
 *
 * @details
 * @todo 1.最近点匹配不用暴力搜索，可以考虑换成KD树
 */
pc LaserOdom::ClosestMatch(pc src, pc tar)
{
    int point_num = src.cols();
    int tar_point_num = tar.cols();
    pc tar_matched;
    tar_matched.resize(2, point_num);

    Eigen::Vector2d src_point, point_diff;
    row_vector dist_list;
    dist_list.resize(1, tar_point_num);
    Eigen::MatrixXd::Index min_index;
    for (int i = 0; i < point_num; i++)
    {
        src_point = src.col(i);
        for (int j = 0; j < tar_point_num; j++)
        {
            point_diff = tar.col(j) - src_point;
            dist_list(j) = point_diff.norm();
        }
        dist_list.minCoeff(&min_index);
        tar_matched.col(i) = tar.col(min_index);
    }
    return tar_matched;
}

/**
 * @brief 评估匹配结果，用于主循环的终止条件
 *
 * @param R
 * @param t
 * @param src
 * @param tar
 * @return double
 * @version 0.1
 * @author jeong (lzh_jeong@qq.com)
 * @date 2022-01-31
 *
 * @details
 * @todo
 */
double LaserOdom::MatchScoreEvaluate(Eigen::Matrix2d R, Eigen::Vector2d t, pc src, pc tar)
{
    pc src_trans, point_diff;
    int point_num = src.cols();
    src_trans.resize(2, point_num);
    point_diff.resize(2, point_num);
    src_trans = R * src + t.replicate(1, point_num);
    double score = 0;
    for (int i = 0; i < point_num; i++)
    {
        score += point_diff.col(i).norm();
    }
    score /= point_num;
    return score;
}