#include "imu_node.hpp"

IMUNode::IMUNode(int32_t const &FS_G, int32_t const &FS_XL, int32_t const &FS_M) : ros::NodeHandle("~"), m_berry_imu(FS_G, FS_XL, FS_M)
{
    m_imu_publisher = advertise<uvc::imu>("imu", 1000);
}

void IMUNode::publish()
{
    Eigen::Vector3d const gyr = m_berry_imu.readGyr();
    Eigen::Vector3d const acc = m_berry_imu.readAcc();
    Eigen::Vector3d const mag = m_berry_imu.readMag();

    uvc::imu imu_msg;

    imu_msg.gyr[0] = gyr.x();
    imu_msg.gyr[1] = gyr.y();
    imu_msg.gyr[2] = gyr.z();

    imu_msg.acc[0] = acc.x();
    imu_msg.acc[1] = acc.y();
    imu_msg.acc[2] = acc.z();

    imu_msg.mag[0] = mag.x();
    imu_msg.mag[1] = mag.y();
    imu_msg.mag[2] = mag.z();

    m_imu_publisher.publish(imu_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_node");

    IMUNode imu_node;
    
    while (ros::ok()) {
        imu_node.publish();

        ros::spinOnce();
    }

    return 0;
}