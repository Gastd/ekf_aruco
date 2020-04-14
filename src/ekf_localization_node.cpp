#include "ekf_aruco/ekf.h"
#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_navigation_node");

    EKF ekf;

    ekf.run();

    return 0;
}
