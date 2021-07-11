#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/imu.h"

#include "berry_imu.hpp"

class IMUNode : public ros::NodeHandle
{
    BerryIMU m_berry_imu;

    Eigen::Vector3d m_gyr_bias;
    Eigen::Vector3d m_acc_bias;
    Eigen::Vector3d m_mag_bias;
    
    ros::Publisher m_imu_publisher;

public:
    IMUNode(int32_t const &FS_G, int32_t const &FS_XL, int32_t const &FS_M);
    
    void publish();
};
