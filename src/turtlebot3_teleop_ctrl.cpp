/**
 * @file turtlebot3_teleop_ctrl.cpp
 * @author jeong (lzh_jeong@qq.com)
 * @brief
 * @version 0.1
 * @date 2022-01-25
 *
 * @details
 * @todo
 */
#include "turtlebot3_teleop_ctrl.h"

#define BURGER_MAX_LIN_VEL 0.22
#define BURGER_MAX_ANG_VEL 2.84

#define WAFFLE_MAX_LIN_VEL 0.26
#define WAFFLE_MAX_ANG_VEL 1.82

#define LIN_VEL_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.1

#define MID_LIN_VEL 0.15
#define MID_ANG_VEL (M_PI / 4)

#define KEY_w 119
#define KEY_W 87
#define KEY_s 115
#define KEY_S 83
#define KEY_a 97
#define KEY_A 65
#define KEY_d 100
#define KEY_D 68
#define KEY_x 120
#define KEY_X 88
#define KEY_SPACE 20

geometry_msgs::Twist robot_twist_cmd;

/**
 * @brief 用ASCII码的形式返回按下的字符
 *
 * @return int
 * @version 0.1
 * @author jeong (lzh_jeong@qq.com)
 * @date 2022-01-25
 *
 * @details
 * @todo
 */
int scanKeyboard()
{
    int in;
    termios new_settings;
    termios stored_settings;
    tcgetattr(STDIN_FILENO, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO); //不在终端里返回显示
    // new_settings.c_lflag &= ~(ICANON);
    new_settings.c_cc[VTIME] = 0; //只取一个字符
    tcgetattr(STDIN_FILENO, &stored_settings);
    new_settings.c_cc[VMIN] = 1; //只取一个字符
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    in = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
    return in;
}

int main(int argc, char **argv)
{
    bool is_not_break = true;
    ros::init(argc, argv, "turtlebot3_teleop_ctrl");
    ros::NodeHandle node_handle;
    ros::Publisher tele_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_twist_cmd.angular.x = robot_twist_cmd.angular.y = robot_twist_cmd.angular.z = 0;
    robot_twist_cmd.linear.x = robot_twist_cmd.linear.y = robot_twist_cmd.linear.z = 0;
    tele_pub.publish(robot_twist_cmd);

    std::cout << "----------------force stop: ctrl+c and press space" << std::endl;
    int count = 0;

    while (is_not_break && ros::ok())
    {
        // std::cout << ":" << scanKeyboard() << std::endl;
        switch (scanKeyboard())
        {
        case KEY_W:
        case KEY_w:
            robot_twist_cmd.linear.x = MID_LIN_VEL;
            robot_twist_cmd.linear.y = robot_twist_cmd.linear.z = 0;
            break;
        case KEY_S:
        case KEY_s:
            robot_twist_cmd.linear.x = robot_twist_cmd.linear.y = robot_twist_cmd.linear.z = 0;
            robot_twist_cmd.angular.x = robot_twist_cmd.angular.y = robot_twist_cmd.angular.z = 0;
            break;
        case KEY_A:
        case KEY_a:
            robot_twist_cmd.angular.z += ANG_VEL_STEP_SIZE;
            robot_twist_cmd.angular.x = robot_twist_cmd.angular.y = 0;
            break;
        case KEY_D:
        case KEY_d:
            robot_twist_cmd.angular.z -= ANG_VEL_STEP_SIZE;
            robot_twist_cmd.angular.x = robot_twist_cmd.angular.y = 0;
            break;
        case KEY_X:
        case KEY_x:
            robot_twist_cmd.linear.x = -MID_LIN_VEL;
            robot_twist_cmd.linear.y = robot_twist_cmd.linear.z = 0;
            break;
        case KEY_SPACE:
            is_not_break = false;
            robot_twist_cmd.angular.x = robot_twist_cmd.angular.y = robot_twist_cmd.angular.z = 0;
            robot_twist_cmd.linear.x = robot_twist_cmd.linear.y = robot_twist_cmd.linear.z = 0;
            break;
        default:
            break;
        }
        if (is_not_break)
        {
            ROS_INFO_STREAM("robot velocity state:---linear:" << robot_twist_cmd.linear.x << "---angular:" << robot_twist_cmd.angular.z);
            tele_pub.publish(robot_twist_cmd);
            count++;
        }
        if (count == 10)
        {
            std::cout << "----------------force stop: ctrl+c and press space" << std::endl;
            count = 0;
        }
    }
    ros::shutdown();
    return 0;
}