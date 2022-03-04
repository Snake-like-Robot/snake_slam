#include <mapping.h>
#include <cmath>
using namespace snake_map;
using namespace std;
SnakeMap::SnakeMap(int lenx, int leny, double xyr)
{
    xw = lenx;
    yw = leny;
    xyreso = xyr;
    width_x = xw * xyreso;
    width_y = yw * xyreso;
    pmap.setIdentity(xw, yw);
    pmap = pmap / 2.0;
    logm.setZero(xw, yw);
    minx = -width_x / 2.0;
    maxx = -minx;
    miny = -width_y / 2.0;
    maxy = -miny;
    rviz_map.header.frame_id = "map";
    rviz_map.header.stamp = ros::Time::now();
    rviz_map.info.resolution = xyreso;
    rviz_map.info.width = xw;
    rviz_map.info.height = yw;
    rviz_map.info.origin.position.x = minx;
    rviz_map.info.origin.position.y = miny;
    rviz_map.info.origin.position.z = 0;
    rviz_map.info.origin.orientation.x = 0;
    rviz_map.info.origin.orientation.y = 0;
    rviz_map.info.origin.orientation.z = 0;
    rviz_map.info.origin.orientation.w = 1;
    rviz_map.data.resize(xw * yw);
    for (int i = 0; i < xw * yw; i++)
        rviz_map.data[i] = -1;
}
int SnakeMap::getindexX(double x) { return (int)((x - minx) / xyreso); }
int SnakeMap::getindexY(double y) { return (int)((y - miny) / xyreso); }
int SnakeMap::MapIndex(int x, int y) { return x * rviz_map.info.width + y; }
void SnakeMap::gridset(int x, int y, bool state)
{
    if (x < 0 || x >= xw || y < 0 || y >= yw)
        return;
#if Bayes
    double tag;
    if (state)
        tag = looccu;
    else
        tag = lofree;

    logm(x, y) += tag;
    pmap(x, y) = 1 - 1.0 / (1 + pow(M_E, logm(x, y)));
#else
    pmap(x, y) = state;
#endif
    // modified 1
    rviz_map.data[MapIndex(y, x)] = (int8_t)(pmap(x, y) * 100);
}
void SnakeMap::bresenham(int x0, int y0, int x1, int y1)
{
    if (x0 < 0 || x0 >= xw || y0 < 0 || y0 >= yw)
        return;
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2, e2;
    for (;;)
    {
        if (x0 == x1 && y0 == y1)
            return;
        if (x0 < 0 || x0 >= xw || y0 < 0 || y0 >= yw)
            return;
        gridset(x0, y0, Free);
        e2 = err;
        if (e2 > -dx)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y0 += sy;
        }
    }
}
int SnakeMap::update(Eigen::VectorXd ox, Eigen::VectorXd oy, double center_x, double center_y)
{
    int x, y, cx = getindexX(center_x), cy = getindexY(center_y);
    for (int i = 0; i < ox.size(); i++)
    {
        x = getindexX(ox(i));
        y = getindexY(oy(i));
        gridset(x, y, Occupy);
        // cout<<cx<<' '<<cy<<' '<<x<<' '<<y<<endl;
        bresenham(cx, cy, x, y);
    }
    return 0;
}
int SnakeMap::update(laser_odom::pc oxy, Eigen::Vector2d center)
{
    return update(oxy.row(0), oxy.row(1), center(0), center(1));
}
Eigen::Vector2i SnakeMap::getMapIndex(Eigen::Vector2d tag)
{
    Eigen::Vector2i p;
    p(0) = getindexX(tag(0));
    p(1) = getindexY(tag(1));
    return p;
}
Eigen::Vector2d SnakeMap::getMapGrid(Eigen::Vector2i tag)
{
    Eigen::Vector2d p;
    p(0) = minx + tag(0) * xyreso + xyreso / 2;
    p(1) = miny + tag(1) * xyreso + xyreso / 2;
    return p;
}
#if _TEST_
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping");
    ros::NodeHandle map_handle;
    ros::Publisher chatter = map_handle.advertise<nav_msgs::OccupancyGrid>("/slam_map", 1);

    SnakeMap map_test(20, 20, 0.1);

    Eigen::VectorXd ox(21), oy(21);
    for (int i = 0; i <= 20; i++)
        ox(i) = (i - 10) / 10.0, oy(i) = ox(i) / 2 + 0.5;
    map_test.update(ox, oy, 0, 0);
    for (int i = 0; i <= 20; i++)
        ox(i) = (i - 10) / 10.0, oy(i) = -ox(i) / 2 - 0.5;
    map_test.update(ox, oy, 0, 0);
    for (int i = 0; i <= 20; i++)
        ox(i) = 0.9, oy(i) = (i - 10) / 10.0;
    map_test.update(ox, oy, 0, 0);

    while (ros::ok())
    {
        chatter.publish(map_test.rviz_map);
    }

    ros::shutdown();
    return 0;
}
#endif