#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/imu.h"

#include "kalman_filter.hpp"

#include <chrono>

class SLAMNode : public ros::NodeHandle
{
    ros::Subscriber m_imu_subscriber;
    
    std::chrono::high_resolution_clock::time_point m_time_point;

    Eigen::Matrix4f m_transform;

public:
    SLAMNode() : ros::NodeHandle("~")
    {
        m_imu_subscriber = subscribe("imu_node/imu", 1000, &SLAMNode::imuCallback, this);
    
        m_transform = Eigen::Matrix4f::Identity();

        m_time_point = std::chrono::high_resolution_clock::now();
    }

    void imuCallback(uvc::gyr::ConstPtr const &msg)
    {
        std::chrono::high_resolution_clock::time_point const time_point = std::chrono::high_resolution_clock::now();

        double const delta_time = std::chrono::duration<double>(time_point - m_time_point).count();

        Eigen::Vector3d delta_rotation = { msg->gyr[0], msg->gyr[1], msg->gyr[2] };
        Eigen::Vector3d delta_position = { msg->acc[0], msg->acc[1], msg->acc[2] };

        // Δr = Δt * Δr'
        delta_rotation *= delta_time;

        // Δp = 1/2 * Δt^2 * Δp''
        delta_position *= delta_time * delta_time * 0.5;

        // Filter out noisy values
        m_kalman_rotation.predict(delta_time);

        Measurement<3> const measure_rotation(delta_rotation, Eigen::Vector4d::Constant(0.03).asDiagonal());
        
        m_kalman_rotation.update(measure_rotation);

        // Filter out noisy values
        m_kalman_position.predict(delta_time);

        Measurement<3> const measure_position(delta_position, Eigen::Vector3d::Constant(0.1).asDiagonal());

        m_kalman_position.update(measure_position);

        delta_rotation = m_kalman_rotation.state();
        delta_position = m_transform.block<3, 3>(0, 0) * m_kalman_position.state();
        
        float cos_x = std::cos(delta_rotation.x());
        float cos_y = std::cos(delta_rotation.y());
        float cos_z = std::cos(delta_rotation.z());
        float sin_x = std::sin(delta_rotation.x());
        float sin_y = std::sin(delta_rotation.y());
        float sin_z = std::sin(delta_rotation.z());

        Eigen::Matrix4f delta_transform;

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

        m_time_point = time_point;
    }
};