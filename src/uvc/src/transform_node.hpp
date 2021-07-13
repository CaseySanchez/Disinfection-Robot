#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/imu.h"

class TransformNode : public ros::NodeHandle
{
	ros::Subscriber m_imu_subscriber;

	double m_gyr_cutoff;
	double m_acc_cutoff;

	KalmanFilter<3> m_kalman_gyr;
	KalmanFilter<3> m_kalman_acc;

	Eigen::Matrix4d m_transform;

public:
	TransformNode();

	Eigen::Matrix4d transform() const;

	void reset();

	void imu(uvc::imu::ConstPtr const &imu);
};