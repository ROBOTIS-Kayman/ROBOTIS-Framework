#include <ros/ros.h>

#include <math.h>

#include <unistd.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

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

class CM730
{
public :
	enum
	{
		P_MODEL_NUMBER_L		= 0,
		P_MODEL_NUMBER_H		= 1,
		P_VERSION				= 2,
		P_ID					= 3,
		P_BAUD_RATE				= 4,
		P_RETURN_DELAY_TIME		= 5,
		P_RETURN_LEVEL			= 16,
		P_DXL_POWER				= 24,
		P_LED_PANNEL			= 25,
		P_LED_HEAD_L			= 26,
		P_LED_HEAD_H			= 27,
		P_LED_EYE_L				= 28,
		P_LED_EYE_H				= 29,
		P_BUTTON				= 30,
		P_GYRO_Z_L				= 38,
		P_GYRO_Z_H				= 39,
		P_GYRO_Y_L				= 40,
		P_GYRO_Y_H				= 41,
		P_GYRO_X_L				= 42,
		P_GYRO_X_H				= 43,
		P_ACCEL_X_L				= 44,
		P_ACCEL_X_H				= 45,
		P_ACCEL_Y_L				= 46,
		P_ACCEL_Y_H				= 47,
		P_ACCEL_Z_L				= 48,
		P_ACCEL_Z_H				= 49,
		P_VOLTAGE				= 50,
		P_LEFT_MIC_L			= 51,
		P_LEFT_MIC_H			= 52,
		P_ADC2_L				= 53,
		P_ADC2_H				= 54,
		P_ADC3_L				= 55,
		P_ADC3_H				= 56,
		P_ADC4_L				= 57,
		P_ADC4_H				= 58,
		P_ADC5_L				= 59,
		P_ADC5_H				= 60,
		P_ADC6_L				= 61,
		P_ADC6_H				= 62,
		P_ADC7_L				= 63,
		P_ADC7_H				= 64,
		P_ADC8_L				= 65,
		P_ADC8_H				= 66,
		P_RIGHT_MIC_L			= 67,
		P_RIGHT_MIC_H			= 68,
		P_ADC10_L				= 69,
		P_ADC10_H				= 70,
		P_ADC11_L				= 71,
		P_ADC11_H				= 72,
		P_ADC12_L				= 73,
		P_ADC12_H				= 74,
		P_ADC13_L				= 75,
		P_ADC13_H				= 76,
		P_ADC14_L				= 77,
		P_ADC14_H				= 78,
		P_ADC15_L				= 79,
		P_ADC15_H				= 80,
		MAXNUM_ADDRESS
	};

	enum
	{
		ID_CM			= 200,
		ID_BROADCAST	= 254
	};
};

class RobotisManager
{

public:
	RobotisManager(ros::NodeHandle nh, ros::NodeHandle param_nh);
	~RobotisManager();
	void onInit();
	void comm_thread_proc();
	void ros_thread_proc();

	static const double G_ACC = 9.80665;

protected:
	ros::NodeHandle nh_, param_nh_;
	std::string name_;
	RobotisController   *controller;
	boost::shared_ptr<GroupHandler>        grp_handler;

	//pthread_mutex_t     mutex = PTHREAD_MUTEX_INITIALIZER;
	boost::mutex mutex;

	int msg_seq_index;
	int syncwrite_addr;
	int syncwrite_data_length;
	std::vector <unsigned char> syncwrite_param;

	std::vector <int>           publish_list;

	ros::Subscriber publish_position_sub;
	ros::Subscriber control_write_sub;
	ros::Subscriber control_torque_sub;
	ros::Subscriber joint_states_sub;
	boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;

	sensor_msgs::Imu imu_data;

	// pthread_t comm_thread;
	boost::thread comm_thread;
	boost::thread ros_thread;

	ros::CallbackQueue my_queue;

	int get_id_from_name(const char* name);
	void publish_localization_tf();
	void publish_imu(ros::Publisher &pub);
	double lowPassFilter(double alpha, double x_new, double x_old);
	void publish_position_callback(const robotis_controller::PublishPosition::ConstPtr& msg);
	void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
	void control_write_callback(const robotis_controller::ControlWrite::ConstPtr& msg);
	void control_torque_callback(const robotis_controller::ControlTorque::ConstPtr& msg);
};

};   // namespace robotis_framework
