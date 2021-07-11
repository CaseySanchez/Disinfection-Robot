#include "imu_node.hpp"

IMUNode::IMUNode(int32_t const &FS_G, int32_t const &FS_XL, int32_t const &FS_M) : ros::NodeHandle("~"), m_berry_imu(FS_G, FS_XL, FS_M)
{
	m_gyr_bias = Eigen::Vector3d::Zero();
	m_acc_bias = Eigen::Vector3d::Zero();
	
	Eigen::Vector3d gyr_bias;
	Eigen::Vector3d acc_bias;
	Eigen::Vector3d mag_bias;
	
	for (size_t i = 0; i < 32; ++i) {
		m_berry_imu.readGyr(gyr_bias.data());
		m_berry_imu.readAcc(acc_bias.data());
		m_berry_imu.readMag(mag_bias.data());
			
		m_gyr_bias += gyr_bias;
		m_acc_bias += acc_bias;
		m_mag_bias += mag_bias;
	}
	
	m_gyr_bias *= 1.0 / 32.0;
	m_acc_bias *= 1.0 / 32.0;
	m_mag_bias *= 1.0 / 32.0;
	
    m_imu_publisher = advertise<uvc::imu>("imu", 1000);
}

void IMUNode::publish()
{
    uvc::imu imu_msg;
	
	m_berry_imu.readGyr(msg.gyr);
	m_berry_imu.readAcc(msg.acc);
	m_berry_imu.readMag(msg.mag);

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
