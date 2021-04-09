#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/gyr.h"
#include "uvc/acc.h"
#include "uvc/mag.h"

#include "berry_imu.hpp"

class IMUNode : public ros::NodeHandle
{
    BerryIMU m_berry_imu;
    
    ros::Publisher m_gyr_publisher;
    ros::Publisher m_acc_publisher;
    ros::Publisher m_mag_publisher;

public:
    IMUNode(int32_t const &FS_G, int32_t const &FS_XL, int32_t const &FS_M);

    void publish();
};