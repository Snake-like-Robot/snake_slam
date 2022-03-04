#ifndef _MAPPING_H_
#define _MAPPING_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include "laser_odometer.h"
#define Occupy true
#define Free false
/*采用贝叶斯概率或暴力概率*/
#define Bayes true
/*本地测试，是否在本地开启一个ros节点，手动输入数据仿真*/
#define _TEST_ false
/*free情况下的概率态，值为ln(0.2/0.8)*/
#define lofree -1.3863
/*occupy情况下的概率态，值为ln(0.9/0.1)*/
#define looccu 2.1972
/*地图旋转，表示逆时针旋转90*MapRotate度*/
#define MapRotate 1
namespace snake_map
{
    class SnakeMap
    {
    private:
        double width_x, width_y, xyreso; //实际大小以及像素精度
        double minx, maxx, miny, maxy;   //边界条件
        uint64_t xw, yw;
        void gridset(int x, int y, bool state);
        void bresenham(int cx, int cy, int ox, int oy);
        int MapIndex(int x, int y);
        int getindexX(double x);
        int getindexY(double y);
        Eigen::MatrixXd pmap, logm;

    public:
        SnakeMap(int lenx, int leny, double xyreso);
        //初始化类，整数lenx，leny表示地图大小（多少格），xyreso表示精度，即每格多宽
        int update(Eigen::VectorXd ox, Eigen::VectorXd oy, double center_x, double center_y);
        int update(laser_odom::pc oxy, Eigen::Vector2d center);
        //向其中添加一世界坐标系下的激光数据，分别为激光点坐标x向量，坐标y向量，以及小车的x，y坐标
        Eigen::Vector2i getMapIndex(Eigen::Vector2d tag);
        Eigen::Vector2d getMapGrid(Eigen::Vector2i tag);
        ~SnakeMap(){};
        nav_msgs::OccupancyGrid rviz_map;
    };
}
#endif