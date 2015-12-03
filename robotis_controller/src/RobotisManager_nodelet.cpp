#include <ros/ros.h>
#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "robotis_controller/RobotisManager.h"

namespace robotis_framework {

class RobotisManagerNodelet : public nodelet::Nodelet {
  public:
    RobotisManagerNodelet() {}

    void onInit() {
      ros::NodeHandle node = getNodeHandle();
      ros::NodeHandle pnode = getPrivateNodeHandle();

      manager = new RobotisManager(node, pnode);
    }

    ~RobotisManagerNodelet() {
      if (manager) delete manager;
    }

  private:
    RobotisManager *manager;
};

};

PLUGINLIB_DECLARE_CLASS(robotis_framework, RobotisManagerNodelet, robotis_framework::RobotisManagerNodelet, nodelet::Nodelet);
