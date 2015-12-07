#include <ros/ros.h>

#include <unistd.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller/ControlWrite.h"
#include "robotis_controller/ControlTorque.h"
#include "robotis_controller/PublishPosition.h"

#include "../../robotis_controller/include/handler/GroupHandler.h"
#include "../../robotis_controller/include/RobotisController.h"

#include <boost/thread.hpp>

#include <ros/callback_queue.h>

using namespace ROBOTIS;

namespace robotis_framework
{

class RobotisManager
{

public:
    RobotisManager(ros::NodeHandle nh, ros::NodeHandle param_nh);
    ~RobotisManager();
    void onInit();
    void comm_thread_proc();
    void ros_thread_proc();

protected:
    ros::NodeHandle nh_, param_nh_;
    std::string name_;
    RobotisController   *controller;
    boost::shared_ptr<GroupHandler>        grp_handler;

    //pthread_mutex_t     mutex = PTHREAD_MUTEX_INITIALIZER;
    boost::mutex mutex;

    int                 syncwrite_addr;
    int                 syncwrite_data_length;
    std::vector <unsigned char> syncwrite_param;

    std::vector <int>           publish_list;

    ros::Subscriber publish_position_sub;
    ros::Subscriber control_write_sub;
    ros::Subscriber control_torque_sub;
    ros::Subscriber joint_states_sub;

    // pthread_t comm_thread;
    boost::thread comm_thread;
    boost::thread ros_thread;

    ros::CallbackQueue my_queue;

    int get_id_from_name(const char* name);
    void publish_position_callback(const robotis_controller::PublishPosition::ConstPtr& msg);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void control_write_callback(const robotis_controller::ControlWrite::ConstPtr& msg);
    void control_torque_callback(const robotis_controller::ControlTorque::ConstPtr& msg);
};

};   // namespace robotis_framework
