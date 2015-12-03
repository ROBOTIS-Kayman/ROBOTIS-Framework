#include <ros/ros.h>
#include <nodelet/loader.h>

#include "robotis_controller/RobotisManager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotis_manager");

    robotis_framework::RobotisManager manager(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();

    return 0;
}
