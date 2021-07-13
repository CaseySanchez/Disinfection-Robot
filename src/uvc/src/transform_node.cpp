#include "transform_node.hpp"

TransformNode::TransformNode() : ros::NodeHandle("~"), m_gyr_cutoff(0.01), m_acc_cutoff(0.01), m_kalman_gyr(Eigen::Matrix<double, 3, 1>::Constant(0.01)), m_kalman_acc(Eigen::Matrix<double, 3, 1>::Constant(0.05))
{
	m_imu_subscriber = subscribe("/imu", 1000, &TransformNode::imu, this);

	m_time_point = std::chrono::high_resolution_clock::now();
}

Eigen::Matrix4d TransformNode::transform() const
{
	return m_transform;
}

void TransformNode::reset()
{
	m_transform = Eigen::Matrix4d::Identity();
}

void TransformNode::imu(uvc::imu::ConstPtr const &imu)
{
	std::chrono::high_resolution_clock::time_point const time_point = std::chrono::high_resolution_clock::now();

	double const delta_time = std::chrono::duration<double>(time_point - m_time_point).count();

	m_time_point = time_point;

	// Filter out noisy values

	Eigen::Vector3d gyr = { imu->gyr[0], imu->gyr[1], imu->gyr[2] };
	Eigen::Vector3d gyr_bias = { imu->gyr_bias[0], imu->gyr_bias[1], imu->gyr_bias[2] };

	m_kalman_gyr.predict(delta_time);

	Measurement<3> const measure_gyr(gyr, Eigen::Vector3d::Constant(0.1).asDiagonal());

	m_kalman_gyr.update(measure_gyr);

	gyr = m_kalman_gyr.state() - gyr_bias;

	if (gyr.norm() < m_gyr_cutoff) {
		gyr = Eigen::Vector3d::Zero();
	}

	// Integrate rotation

	Eigen::Vector3d delta_rotation_angle_axis = gyr * delta_time;

	double cos_x = std::cos(delta_rotation_angle_axis.x());
	double cos_y = std::cos(delta_rotation_angle_axis.y());
	double cos_z = std::cos(delta_rotation_angle_axis.z());
	double sin_x = std::sin(delta_rotation_angle_axis.x());
	double sin_y = std::sin(delta_rotation_angle_axis.y());
	double sin_z = std::sin(delta_rotation_angle_axis.z());

	Eigen::Matrix3d delta_rotation;

	delta_rotation(0, 0) = cos_x * cos_y;
	delta_rotation(0, 1) = cos_x * sin_y * sin_z - sin_x * cos_z;
	delta_rotation(0, 2) = cos_x * sin_y * cos_z + sin_x * sin_z;
	delta_rotation(1, 0) = sin_x * cos_y;
	delta_rotation(1, 1) = sin_x * sin_y * sin_z + cos_x * cos_z;
	delta_rotation(1, 2) = sin_x * sin_y * cos_z - cos_x * sin_z;
	delta_rotation(2, 0) = -sin_y;
	delta_rotation(2, 1) = cos_y * sin_z;
	delta_rotation(2, 2) = cos_y * cos_z;

	m_rotation *= delta_rotation;

	// Filter out noisy values

	Eigen::Vector3d acc = { imu->acc[0], imu->acc[1], imu->acc[2] };
	Eigen::Vector3d acc_bias = { imu->acc_bias[0], imu->acc_bias[1], imu->acc_bias[2] };

	m_kalman_acc.predict(delta_time);

	Measurement<3> const measure_acc(acc, Eigen::Vector3d::Constant(0.1).asDiagonal());

	m_kalman_acc.update(measure_acc);

	Eigen::Vector3d acc = m_kalman_acc.state() - m_rotation.inverse() * acc_bias;

	if (acc.norm() < m_acc_cutoff) {
		acc = Eigen::Vector3d::Zero();
	}

	// Integrate position

	Eigen::Vector3d delta_position = m_rotation * acc * delta_time * delta_time * 0.5;

	m_position += delta_position;

	Eigen::Matrix4d delta_transform;

	delta_transform(0, 0) = cos_x * cos_y;
	delta_transform(0, 1) = cos_x * sin_y * sin_z - sin_x * cos_z;
	delta_transform(0, 2) = cos_x * sin_y * cos_z + sin_x * sin_z;
	delta_transform(0, 3) = delta_position.x();
	delta_transform(1, 0) = sin_x * cos_y;
	delta_transform(1, 1) = sin_x * sin_y * sin_z + cos_x * cos_z;
	delta_transform(1, 2) = sin_x * sin_y * cos_z - cos_x * sin_z;
	delta_transform(1, 3) = delta_position.y();
	delta_transform(2, 0) = -sin_y;
	delta_transform(2, 1) = cos_y * sin_z;
	delta_transform(2, 2) = cos_y * cos_z;
	delta_transform(2, 3) = delta_position.z();
	delta_transform(3, 0) = 0.0f;
	delta_transform(3, 1) = 0.0f;
	delta_transform(3, 2) = 0.0f;
	delta_transform(3, 3) = 1.0f;

	m_transform *= delta_transform;
}