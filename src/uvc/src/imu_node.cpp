#include "imu_node.hpp"

IMUNode::IMUNode(int32_t const &FS_G, int32_t const &FS_XL, int32_t const &FS_M) : ros::NodeHandle("~"), m_berry_imu(FS_G, FS_XL, FS_M)
{
    m_gyr_publisher = advertise<uvc::gyr>("gyr", 1000);
    m_acc_publisher = advertise<uvc::acc>("acc", 1000);
    m_mag_publisher = advertise<uvc::mag>("mag", 1000);
}

void IMUNode::publish()
{
    Eigen::Vector3d const gyr = m_berry_imu.readGyr();
    Eigen::Vector3d const acc = m_berry_imu.readAcc();
    Eigen::Vector3d const mag = m_berry_imu.readMag();

    uvc::gyr gyr_msg;
    uvc::acc acc_msg;
    uvc::mag mag_msg;

    gyr_msg.x = gyr.x();
    gyr_msg.y = gyr.y();
    gyr_msg.z = gyr.z();

    acc_msg.x = acc.x();
    acc_msg.y = acc.y();
    acc_msg.z = acc.z();

    mag_msg.x = mag.x();
    mag_msg.y = mag.y();
    mag_msg.z = mag.z();

    m_gyr_publisher.publish(gyr_msg);
    m_acc_publisher.publish(acc_msg);
    m_mag_publisher.publish(mag_msg);
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