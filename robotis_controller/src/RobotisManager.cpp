/*
 * RobotisManager.cpp
 *
 *  Created on: 2015. 11. 18.
 *      Author: zerom
 */

#include "robotis_controller/RobotisManager.h"

using namespace ROBOTIS;

namespace robotis_framework
{

RobotisManager::RobotisManager(ros::NodeHandle nh, ros::NodeHandle param_nh)
: nh_(nh), param_nh_(param_nh)
{
	controller = new RobotisController();
	grp_handler.reset(new GroupHandler(controller));
	tf_broadcaster_.reset(new tf::TransformBroadcaster());

	// call queue
	// nh_.setCallbackQueue(&my_queue);

	// mutex = PTHREAD_MUTEX_INITIALIZER;

	publish_position_sub    = nh_.subscribe("/publish_position", 10, &RobotisManager::publish_position_callback, this);
	control_write_sub       = nh_.subscribe("/control_write", 10, &RobotisManager::control_write_callback, this);
	control_torque_sub      = nh_.subscribe("/control_torque", 10, &RobotisManager::control_torque_callback, this);


	std::string topic_name;
	if(param_nh_.getParam("subscribe_joint_topic_name", topic_name) == false)
		topic_name = "/controller_joint_states";

	/*
    // use user callback queue
    ros::SubscribeOptions _queue_option = ros::SubscribeOptions::create<sensor_msgs::JointState>
    				(topic_name
    				, 10
					, boost::bind(static_cast<void (RobotisManager::*)
							(const sensor_msgs::JointState::ConstPtr&)>(&RobotisManager::joint_states_callback), this, _1)
    				, ros::VoidPtr()
    				, &this->my_queue);
    joint_states_sub = nh_.subscribe(_queue_option);
	 */

	// Default
	joint_states_sub = nh_.subscribe(topic_name, 10, &RobotisManager::joint_states_callback, this);


	//pthread_create(&comm_thread, 0, &RobotisManager::comm_thread_proc, this);
	// ros_thread = boost::thread(boost::bind(&RobotisManager::ros_thread_proc, this));
	comm_thread = boost::thread(boost::bind(&RobotisManager::comm_thread_proc, this));

	// ros::AsyncSpinner _spinner(0, &my_queue);
	// _spinner.start();
	// ros::MultiThreadedSpinner spinner(0);
	// spinner.spin(&my_queue);
}

RobotisManager::~RobotisManager()
{
	if(comm_thread.joinable()) comm_thread.join();
	if(ros_thread.joinable()) ros_thread.join();
}

int RobotisManager::get_id_from_name(const char* name)
{
	int id = -1;
	for(int i = 0; i < controller->idList.size(); i++)
	{
		id = controller->idList[i];
		if(strcmp(controller->getDevice(id)->getJointName(), name) == 0)
			return id;
	}
	return -1;
}

void RobotisManager::publish_localization_tf()
{
	geometry_msgs::TransformStamped _transform;

	_transform.header.stamp = imu_data.header.stamp;
	_transform.header.frame_id = "map";
	_transform.child_frame_id = "base_link";
	_transform.transform.rotation.w = imu_data.orientation.w;
	//ROS and ROBITIS x and y frame definitions deviate by 90 degree around z axis
	_transform.transform.rotation.x = imu_data.orientation.x;
	_transform.transform.rotation.y = imu_data.orientation.y;
	_transform.transform.rotation.z = imu_data.orientation.z;
	tf_broadcaster_->sendTransform(_transform);
}

void RobotisManager::publish_imu(ros::Publisher &pub)
{
	imu_data.header.stamp = ros::Time::now();
	imu_data.header.frame_id = "body_link";

	//	double imu_orientation[4];
	double imu_angular_velocity[3];
	double imu_linear_acceleration[3];
	double filter_alpha = 0.4;

	//in rad/s
	long int _value = 0;
	int _arrd_length = 2;
	grp_handler->getReadData(CM730::ID_CM, CM730::P_GYRO_X_L, (long int*)&_value, _arrd_length);
	imu_angular_velocity[0] = _value;
	grp_handler->getReadData(CM730::ID_CM, CM730::P_GYRO_Y_L, (long int*)&_value, _arrd_length);
	imu_angular_velocity[1] = _value;
	grp_handler->getReadData(CM730::ID_CM, CM730::P_GYRO_Z_L, (long int*)&_value, _arrd_length);
	imu_angular_velocity[2] = _value;
	imu_data.angular_velocity.x = lowPassFilter(filter_alpha, (imu_angular_velocity[1] - 512) * 1600.0 * M_PI / (512.0 * 180.0), imu_data.angular_velocity.x);
	imu_data.angular_velocity.y = lowPassFilter(filter_alpha, (imu_angular_velocity[0] - 512) * 1600.0 * M_PI / (512.0 * 180.0), imu_data.angular_velocity.y);
	imu_data.angular_velocity.z = lowPassFilter(filter_alpha, (imu_angular_velocity[2] - 512) * 1600.0 * M_PI / (512.0 * 180.0), imu_data.angular_velocity.z);
	// ROS_INFO("angular velocity : %f, %f, %f", imu_angular_velocity[0], imu_angular_velocity[1], imu_angular_velocity[2]);

	//in m/s^2
	double _const = 1;
	grp_handler->getReadData(CM730::ID_CM, CM730::P_ACCEL_X_L, (long int*)&_value, _arrd_length);
	imu_linear_acceleration[0] = _value;
	grp_handler->getReadData(CM730::ID_CM, CM730::P_ACCEL_Y_L, (long int*)&_value, _arrd_length);
	imu_linear_acceleration[1] = _value;
	grp_handler->getReadData(CM730::ID_CM, CM730::P_ACCEL_Z_L, (long int*)&_value, _arrd_length);
	imu_linear_acceleration[2] = _value;
	imu_data.linear_acceleration.x = lowPassFilter(filter_alpha, (imu_linear_acceleration[1] - 512) * G_ACC * _const * 4.0 / 512.0, imu_data.linear_acceleration.x);
	imu_data.linear_acceleration.y = lowPassFilter(filter_alpha, (imu_linear_acceleration[0] - 512) * G_ACC * _const * 4.0 / 512.0, imu_data.linear_acceleration.y);
	imu_data.linear_acceleration.z = lowPassFilter(filter_alpha, (imu_linear_acceleration[2] - 512) * G_ACC * _const * 4.0 / 512.0, imu_data.linear_acceleration.z);
	// ROS_INFO("linear_acceleration : %f, %f, %f", imu_linear_acceleration[0], imu_linear_acceleration[1], imu_linear_acceleration[2]);

	//Estimation of roll and pitch based on accelometer data, see http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
	double sign = copysignf(1.0,  imu_data.linear_acceleration.z/G_ACC);
	double roll = atan2( imu_data.linear_acceleration.y/G_ACC, sign * sqrt( imu_data.linear_acceleration.x/G_ACC* imu_data.linear_acceleration.x/G_ACC +  imu_data.linear_acceleration.z/G_ACC* imu_data.linear_acceleration.z/G_ACC));
	double pitch = -atan2( imu_data.linear_acceleration.x/G_ACC, sqrt( imu_data.linear_acceleration.y/G_ACC* imu_data.linear_acceleration.y/G_ACC +  imu_data.linear_acceleration.z/G_ACC* imu_data.linear_acceleration.z/G_ACC));
	double yaw = 0.0;

	tf2::Quaternion imu_orient;
	imu_orient.setRPY(roll, pitch, yaw);

	imu_data.orientation.x = imu_orient.getX();
	imu_data.orientation.y = imu_orient.getY();
	imu_data.orientation.z = imu_orient.getZ();
	imu_data.orientation.w = imu_orient.getW();

	pub.publish(imu_data);
}

double RobotisManager::lowPassFilter(double alpha, double x_new, double x_old)
{
	return alpha*x_new + (1.0-alpha)*x_old;
}

void RobotisManager::publish_position_callback(const robotis_controller::PublishPosition::ConstPtr& msg)
{
	if(msg->name.size() == 0 || msg->name.size() != msg->publish.size())
		return;

	for(int i = 0; i < msg->name.size(); i++)
	{
		int id = get_id_from_name(msg->name[i].c_str());
		if(id == -1)
			continue;

		if(msg->publish[i] == true) {
			grp_handler->pushBulkRead(id, controller->getDevice(id)->ADDR_PRESENT_POSITION);
			if ( std::find(publish_list.begin(), publish_list.end(), id) == publish_list.end() )
				publish_list.push_back(get_id_from_name(msg->name[i].c_str()));
		}
		else {
			grp_handler->deleteBulkRead(id);
			std::vector<int>::iterator iter = std::find(publish_list.begin(), publish_list.end(), id);
			if(iter != publish_list.end())
				publish_list.erase(iter);
		}
	}
}

void RobotisManager::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	int n = 0, id = -1;

	if(msg->name.size() == 0 || msg->name.size() != msg->position.size())
		return;

	//pthread_mutex_lock(&mutex);
	mutex.lock();
	// ros::Duration _dur = ros::Time::now() - msg->header.stamp;
	// ROS_INFO_STREAM("m_received | " << _dur);
	msg_seq_index = msg->header.seq;
	syncwrite_param.clear();
	for(unsigned int idx = 0; idx < msg->name.size(); idx++)
	{
		id = get_id_from_name(msg->name[idx].c_str());
		if(id != -1)
		{
			syncwrite_addr = controller->getDevice(id)->ADDR_GOAL_POSITION;
			syncwrite_data_length = controller->getDevice(id)->getAddrLength(syncwrite_addr);
			int pos     = controller->getDevice(id)->rad2Value(msg->position[idx]);
			syncwrite_param.resize(syncwrite_param.size() + syncwrite_data_length + 1);
			syncwrite_param[n++]  = id;
			if(syncwrite_data_length == 2)
			{
				syncwrite_param[n++]  = DXL_LOBYTE(pos);
				syncwrite_param[n++]  = DXL_HIBYTE(pos);
			}
			else if(syncwrite_data_length == 4)
			{
				syncwrite_param[n++]  = DXL_LOBYTE(DXL_LOWORD(pos));
				syncwrite_param[n++]  = DXL_HIBYTE(DXL_LOWORD(pos));
				syncwrite_param[n++]  = DXL_LOBYTE(DXL_HIWORD(pos));
				syncwrite_param[n++]  = DXL_HIBYTE(DXL_HIWORD(pos));
			}
		}
	}
	// pthread_mutex_unlock(&mutex);
	mutex.unlock();
}

void RobotisManager::control_write_callback(const robotis_controller::ControlWrite::ConstPtr& msg)
{
	switch(msg->length)
	{
	case 1:
		controller->write(msg->id, msg->addr, msg->value, LENGTH_1BYTE, 0);
		break;
	case 2:
		controller->write(msg->id, msg->addr, msg->value, LENGTH_2BYTE, 0);
		break;
	case 4:
		controller->write(msg->id, msg->addr, msg->value, LENGTH_4BYTE, 0);
		break;
	default:
		break;
	}
}

void RobotisManager::control_torque_callback(const robotis_controller::ControlTorque::ConstPtr& msg)
{
	int n = 0, id = -1;

	if(msg->name.size() == 0 || msg->name.size() != msg->enable.size())
		return;

	// pthread_mutex_lock(&mutex);
	mutex.lock();
	syncwrite_param.clear();
	for(int i = 0; i < msg->name.size(); i++)
	{
		id = get_id_from_name(msg->name[i].c_str());
		if(id != -1)
		{
			syncwrite_addr = controller->getDevice(id)->ADDR_TORQUE_ENABLE;
			syncwrite_data_length = controller->getDevice(id)->getAddrLength(syncwrite_addr);
			syncwrite_param.resize(syncwrite_param.size() + syncwrite_data_length + 1); // 2 : ID(1) + TORQUE_ENABLE(1)
			syncwrite_param[n++]  = id;
			if(msg->enable[i] == true)
				syncwrite_param[n++]  = 1;
			else
				syncwrite_param[n++]  = 0;
		}
	}
	// pthread_mutex_unlock(&mutex);
	mutex.unlock();
}

void RobotisManager::comm_thread_proc()
{
	// ros::NodeHandle nh("~");

	ros::Publisher joint_states_pub;
	std::string topic_name;
	if(param_nh_.getParam("publish_joint_topic_name", topic_name) == true)
		joint_states_pub = nh_.advertise<sensor_msgs::JointState>(topic_name, 1);
	else
		joint_states_pub = nh_.advertise<sensor_msgs::JointState>("/robot_joint_states", 1);

	ros::Publisher _imu_pub      = nh_.advertise<sensor_msgs::Imu>("/robot_imu_data", 1);

	ros::Publisher manager_ready_pub = nh_.advertise<std_msgs::Bool>("/manager_ready", 10, true);

	// check .launch file parameter
	if (controller->initialize(param_nh_) == false)
	{
		ROS_ERROR("robotis_controller initialize failed");
		return ;
	}

	std_msgs::Bool ready;
	ready.data = true;
	manager_ready_pub.publish(ready);

	ros::Rate _loop_hz(125);	// 8ms

	ros::Time _time = ros::Time::now();
	int _seq = 0;

	// gyro bulkread
	int _arrd_length = 2;
	grp_handler->pushBulkRead(CM730::ID_CM, CM730::P_GYRO_Z_L, 12);

	while(ros::ok())
	{
		ros::Time _now = ros::Time::now();
		ros::Duration _dur = _now - _time;
		_time = _now;

		int _msg_seq = 0;

		// Run BulkRead
		grp_handler->runBulkRead();

		if(syncwrite_param.size() > 0) {
			//pthread_mutex_lock(&mutex);
			mutex.lock();
			int r = grp_handler->syncWrite(syncwrite_addr, syncwrite_data_length, &syncwrite_param[0], syncwrite_param.size());

			/*
            // log
			_msg_seq = msg_seq_index - _seq;
            _seq = msg_seq_index;
            ROS_INFO_STREAM("loop_rate " << _dur << " | " << _msg_seq);
			 */
			mutex.unlock();
			// pthread_mutex_unlock(&mutex);
		}
		syncwrite_param.clear();

		// publish joint states
		sensor_msgs::JointState joint_states;
		if(publish_list.size() > 0)
		{
			for(int i = 0; i < publish_list.size(); i++)
			{
				int     _pos    = 0;
				int     _id     = publish_list[i];
				if(grp_handler->getReadData(_id, controller->getDevice(_id)->ADDR_PRESENT_POSITION, (long int*)&_pos) == true)
				{
					joint_states.name.push_back(controller->getDevice(_id)->getJointName());
					joint_states.position.push_back(controller->getDevice(_id)->value2Rad(_pos));
				}
			}
			joint_states.header.stamp = ros::Time::now();
			joint_states_pub.publish(joint_states);
		}

		// publish imu data

		//	double imu_orientation[4];
		//		int imu_angular_velocity[3];
		//		int imu_linear_acceleration[3];
		//		double filter_alpha = 0.5;
		//
		//		//in rad/s
		//		long int _value = 0;
		//		int _arrd_length = 2;
		//		grp_handler->getReadData(CM730::ID_CM, CM730::P_GYRO_X_L, &_value, _arrd_length);
		//		imu_angular_velocity[0] = _value;
		//		imu_data.angular_velocity.x = lowPassFilter(filter_alpha, (imu_angular_velocity[0] - 512) * 1600.0 * M_PI / (512.0 * 180.0), imu_data.angular_velocity.x);
		//		grp_handler->getReadData(CM730::ID_CM, CM730::P_GYRO_Y_L, &_value, _arrd_length);
		//		imu_angular_velocity[1] = _value;
		//		imu_data.angular_velocity.y = lowPassFilter(filter_alpha, (imu_angular_velocity[1] - 512) * 1600.0 * M_PI / (512.0 * 180.0), imu_data.angular_velocity.y);
		//		grp_handler->getReadData(CM730::ID_CM, CM730::P_GYRO_Z_L, &_value, _arrd_length);
		//		imu_angular_velocity[2] = _value;
		//		imu_data.angular_velocity.z = lowPassFilter(filter_alpha, (imu_angular_velocity[2] - 512) * 1600.0 * M_PI / (512.0 * 180.0), imu_data.angular_velocity.z);
		//		// ROS_INFO("angular velocity : %f, %f, %f", imu_angular_velocity[0], imu_angular_velocity[1], imu_angular_velocity[2]);
		//
		//		//in m/s^2
		//		double _const = 1;
		//		grp_handler->getReadData(CM730::ID_CM, CM730::P_ACCEL_X_L, &_value, _arrd_length);
		//		imu_linear_acceleration[0] = _value;
		//		imu_data.linear_acceleration.x = lowPassFilter(filter_alpha, (imu_linear_acceleration[0] - 512) * G_ACC * 4.0 / 512.0 * _const, imu_data.linear_acceleration.x);
		//		grp_handler->getReadData(CM730::ID_CM, CM730::P_ACCEL_Y_L, &_value, _arrd_length);
		//		imu_linear_acceleration[1] = _value;
		//		imu_data.linear_acceleration.y = lowPassFilter(filter_alpha, (imu_linear_acceleration[1] - 512) * G_ACC * 4.0 / 512.0 * _const, imu_data.linear_acceleration.y);
		//		grp_handler->getReadData(CM730::ID_CM, CM730::P_ACCEL_Z_L, &_value, _arrd_length);
		//		imu_linear_acceleration[2] = _value;
		//		imu_data.linear_acceleration.z = lowPassFilter(filter_alpha, (imu_linear_acceleration[2] - 512) * G_ACC * 4.0 / 512.0 * _const, imu_data.linear_acceleration.z);
		//		// ROS_INFO("linear_acceleration : %f, %f, %f", imu_linear_acceleration[0], imu_linear_acceleration[1], imu_linear_acceleration[2]);
		//		// imu_data.linear_acceleration.y =  0.1*imu_data.linear_acceleration.y;
		//		// imu_data.linear_acceleration.x =  0.1*imu_data.linear_acceleration.x;
		//		// imu_data.linear_acceleration.z =  0.1*imu_data.linear_acceleration.z;
		//
		//		//Estimation of roll and pitch based on accelometer data, see http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
		//		double sign = copysignf(1.0,  imu_data.linear_acceleration.z/G_ACC);
		//		double roll = atan2( imu_data.linear_acceleration.y/G_ACC, sign * sqrt( imu_data.linear_acceleration.x/G_ACC* imu_data.linear_acceleration.x/G_ACC +  imu_data.linear_acceleration.z/G_ACC* imu_data.linear_acceleration.z/G_ACC));
		//		double pitch = -atan2( imu_data.linear_acceleration.x/G_ACC, sqrt( imu_data.linear_acceleration.y/G_ACC* imu_data.linear_acceleration.y/G_ACC +  imu_data.linear_acceleration.z/G_ACC* imu_data.linear_acceleration.z/G_ACC));
		//		double yaw = 0.0;
		//
		//		tf2::Quaternion imu_orient;
		//		imu_orient.setRPY(roll, pitch, yaw);
		//
		//		imu_data.orientation.x = imu_orient.getX();
		//		imu_data.orientation.y = imu_orient.getY();
		//		imu_data.orientation.z = imu_orient.getZ();
		//		imu_data.orientation.w = imu_orient.getW();

		publish_imu(_imu_pub);
		//publish_localization_tf();

		ros::spinOnce();
		_loop_hz.sleep();


	}

	ROS_INFO("Terminated ROS");
	return;
}

void RobotisManager::ros_thread_proc()
{
	while(ros::ok())
	{
		my_queue.callAvailable(ros::WallDuration(0.01));
		ros::spinOnce();
	}
	return;
}

}
