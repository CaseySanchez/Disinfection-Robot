/*
 * Copyright 2020 Casey Sanchez
 */

#include "berry_imu.hpp"

BerryIMU::BerryIMU(int32_t FS_G, int32_t FS_XL, int32_t FS_M) : m_FS_G(FS_G), m_FS_XL(FS_XL), m_FS_M(FS_M)
{
    m_file = open("/dev/i2c-1", O_RDWR);

    if (m_file < 0) {
        throw std::runtime_error(std::string("Unable to open I2C bus with error: ") + strerror(errno));
    }

    //Detect if BerryIMUv2 (Which uses a LSM9DS1) is connected
    selectDevice(m_file, LSM9DS1_MAG_ADDRESS);

    int32_t LSM9DS1_WHO_M_response = i2c_smbus_read_byte_data(m_file, LSM9DS1_WHO_AM_I_M);

    selectDevice(m_file, LSM9DS1_GYR_ADDRESS);

    int32_t LSM9DS1_WHO_XG_response = i2c_smbus_read_byte_data(m_file, LSM9DS1_WHO_AM_I_XG);

    if (LSM9DS1_WHO_XG_response != 0x68 || LSM9DS1_WHO_M_response != 0x3d) {
        throw std::runtime_error("LSM9DS1 not detected");
    }
    
    // Enable the gyroscope
    writeByte(LSM9DS1_GYR_ADDRESS, LSM9DS1_CTRL_REG4, 0b00111000);      // z, y, x axis enabled for gyro
    writeByte(LSM9DS1_GYR_ADDRESS, LSM9DS1_CTRL_REG1_G, 0b10100000 | (FS_G_bits[m_FS_G] << 3));    // Gyro ODR = 476Hz, 245 dps
    writeByte(LSM9DS1_GYR_ADDRESS, LSM9DS1_ORIENT_CFG_G, 0b10111000);   // Swap orientation 

    // Enable the accelerometer
    writeByte(LSM9DS1_ACC_ADDRESS, LSM9DS1_CTRL_REG5_XL, 0b00111000);   // z, y, x axis enabled for accelerometer
    writeByte(LSM9DS1_ACC_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0b00100000 | (FS_XL_bits[m_FS_XL] << 3));   // +/- 2g

    //Enable the magnetometer
    writeByte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG1_M, 0b10011100);   // Temp compensation enabled,Low power mode mode,80Hz ODR
    writeByte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG2_M, 0b00000000 | (FS_M_bits[m_FS_M] << 5));   // +/-12gauss
    writeByte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG3_M, 0b00000000);   // continuous update
    writeByte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG4_M, 0b00000000);   // lower power mode for Z axis
}

// Convert to SI units [mdeg/sec]->[rad/sec]
Eigen::Vector3d BerryIMU::readGyr()
{
    double constexpr conversion { 3.14159265358979323846 / 180.0 / 1000.0 }; 
    
    int16_t raw[3];
    
    readBytes(LSM9DS1_GYR_ADDRESS, LSM9DS1_OUT_X_L_G, 6, reinterpret_cast<uint8_t *>(&raw[0]));
    
    Eigen::Vector3d gyr;

    gyr[0] = static_cast<double>(raw[0]) * FS_G_sensitivity[m_FS_G] * conversion;
    gyr[1] = static_cast<double>(raw[1]) * FS_G_sensitivity[m_FS_G] * conversion;
    gyr[2] = static_cast<double>(raw[2]) * FS_G_sensitivity[m_FS_G] * conversion;

    return gyr;
}

// Convert to SI units [mG]->[m/s^2]
Eigen::Vector3d BerryIMU::readAcc()
{
    double constexpr conversion { 9.8 / 1000.0 };
    
    int16_t raw[3];
    
    readBytes(LSM9DS1_ACC_ADDRESS, LSM9DS1_OUT_X_L_XL, 6, reinterpret_cast<uint8_t *>(&raw[0]));
    
    Eigen::Vector3d acc;

    acc[0] = static_cast<double>(raw[0]) * FS_XL_sensitivity[m_FS_XL] * conversion;
    acc[1] = static_cast<double>(raw[1]) * FS_XL_sensitivity[m_FS_XL] * conversion;
    acc[2] = static_cast<double>(raw[2]) * FS_XL_sensitivity[m_FS_XL] * conversion;

    return acc;
}

// Convert to SI units [mGauss]->[Tesla]
Eigen::Vector3d BerryIMU::readMag()
{
    double constexpr conversion { 1.0 / 10000.0 / 1000.0 };
    
    int16_t raw[3];
    
    readBytes(LSM9DS1_MAG_ADDRESS, LSM9DS1_OUT_X_L_M, 6, reinterpret_cast<uint8_t *>(&raw[0]));
    
    Eigen::Vector3d mag;

    mag[0] = static_cast<double>(raw[0]) * FS_M_sensitivity[m_FS_M] * conversion;
    mag[1] = static_cast<double>(raw[1]) * FS_M_sensitivity[m_FS_M] * conversion;
    mag[2] = static_cast<double>(raw[2]) * FS_M_sensitivity[m_FS_M] * conversion;

    return mag;
}

Eigen::Vector3d BerryIMU::computeGyrBias()
{
    // Must be less than 32
    uint8_t constexpr samples { 31 };

    writeByte(LSM9DS1_GYR_ADDRESS, LSM9DS1_CTRL_REG9, 0b00000010);
    writeByte(LSM9DS1_GYR_ADDRESS, LSM9DS1_FIFO_CTRL, 0b00100000 | samples);
    
    uint8_t fifo_src_data = 0;
    
    while(fifo_src_data < samples) {
        readByte(LSM9DS1_GYR_ADDRESS, LSM9DS1_FIFO_SRC, &fifo_src_data);
        
        fifo_src_data &= 0x3F;
    }
    
    Eigen::Vector3d gyr_bias = Eigen::Vector3d::Zero();

    for (uint8_t i = 0; i < samples; ++i) {
        gyr_bias += readGyr();
    }
    
    gyr_bias /= static_cast<double>(samples);
    
    writeByte(LSM9DS1_GYR_ADDRESS, LSM9DS1_CTRL_REG9, 0b00000000);
    writeByte(LSM9DS1_GYR_ADDRESS, LSM9DS1_FIFO_CTRL, 0b00000000);

    return gyr_bias;
}

Eigen::Vector3d BerryIMU::computeAccBias()
{
    // Must be less than 32
    uint8_t constexpr samples { 31 };

    writeByte(LSM9DS1_ACC_ADDRESS, LSM9DS1_CTRL_REG9, 0b00000010);
    writeByte(LSM9DS1_ACC_ADDRESS, LSM9DS1_FIFO_CTRL, 0b00100000 | samples);
    
    uint8_t fifo_src_data = 0;
    
    while(fifo_src_data < samples) {
        readByte(LSM9DS1_ACC_ADDRESS, LSM9DS1_FIFO_SRC, &fifo_src_data);
        
        fifo_src_data &= 0x3F;
    }
    
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();

    for (int32_t i = 0; i < samples; ++i) {
        acc_bias += readAcc();
    }
    
    acc_bias /= static_cast<double>(samples);
    
    writeByte(LSM9DS1_ACC_ADDRESS, LSM9DS1_CTRL_REG9, 0b00000000);
    writeByte(LSM9DS1_ACC_ADDRESS, LSM9DS1_FIFO_CTRL, 0b00000000);

    return acc_bias;
}

Eigen::Vector3d BerryIMU::computeMagBias()
{
    // Must be less than 32
    uint8_t constexpr samples { 31 };

    writeByte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG9, 0b00000010);
    writeByte(LSM9DS1_MAG_ADDRESS, LSM9DS1_FIFO_CTRL, 0b00100000 | samples);
    
    uint8_t fifo_src_data = 0;
    
    while(fifo_src_data < samples) {
        readByte(LSM9DS1_MAG_ADDRESS, LSM9DS1_FIFO_SRC, &fifo_src_data);
        
        fifo_src_data &= 0x3F;
    }
    
    Eigen::Vector3d mag_bias = Eigen::Vector3d::Zero();
    
    for (int32_t i = 0; i < samples; ++i) {
        mag_bias += readMag();
    }
    
    mag_bias /= static_cast<double>(samples);

    writeByte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG9, 0b00000000);
    writeByte(LSM9DS1_MAG_ADDRESS, LSM9DS1_FIFO_CTRL, 0b00000000);

    return mag_bias;
}

void BerryIMU::selectDevice(int32_t file, int32_t addr)
{
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        throw std::runtime_error(std::string("Failed to select I2C device with error: ") + strerror(errno));
    }
}

void BerryIMU::readBytes(int32_t addr, uint8_t reg, uint8_t size, uint8_t *data)
{
    selectDevice(m_file, addr);

    int32_t result = i2c_smbus_read_i2c_block_data(m_file, 0x80 | reg, size, data);

    if (result != size) {
        throw std::runtime_error(std::string("Failed to read I2C bytes with error: ") + strerror(errno));
    } 
}

void BerryIMU::readByte(int32_t addr, uint8_t reg, uint8_t *data)
{
    selectDevice(m_file, addr);
    
    *data = i2c_smbus_read_byte_data(m_file, reg);
    
    if (*data == -1) {
        throw std::runtime_error(std::string("Failed to read I2C byte with error: ") + strerror(errno));
    }
}

void BerryIMU::writeByte(int32_t addr, uint8_t reg, uint8_t value)
{
    selectDevice(m_file, addr);

    int32_t result = i2c_smbus_write_byte_data(m_file, reg, value);

    if (result == -1) {
        throw std::runtime_error(std::string("Failed to write I2C byte with error: ") + strerror(errno));
    }
}
