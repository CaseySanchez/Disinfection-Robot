/*
 * Copyright 2020 Casey Sanchez
 */

#pragma once

#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <stdexcept>

#include <eigen3/Eigen/Dense>

#include "i2c-dev.h"
#include "LSM9DS1.h"

enum {
	FS_G_245 = 0,
	FS_G_500,
	FS_G_2000
};

enum {
	FS_XL_2 = 0,
	FS_XL_4,
	FS_XL_8,
	FS_XL_16
};

enum {
	FS_M_4 = 0,
	FS_M_8,
	FS_M_12,
	FS_M_16
};

int32_t const FS_G_bits[3] = {
	0b00,
	0b01,
	0b11
};

double const FS_G_sensitivity[3] = {
	8.75,
	17.5,
	70.0
};

int32_t const FS_XL_bits[4] = {
	0b00,
	0b10,
	0b11,
	0b01
};

double const FS_XL_sensitivity[4] = {
	0.061,
	0.122,
	0.244,
	0.732
};

int32_t const FS_M_bits[4] = {
	0b00,
	0b01,
	0b10,
	0b11
};

double const FS_M_sensitivity[4] = {
	0.14,
	0.29,
	0.43,
	0.58
};

class BerryIMU
{
	int32_t m_file;
	int32_t m_FS_G;
	int32_t m_FS_XL;
	int32_t m_FS_M;

public:
	BerryIMU(int32_t FS_G, int32_t FS_XL, int32_t FS_M);

	// Convert to SI units [mdeg/sec]->[rad/sec]
	Eigen::Vector3d readGyr();
	// Convert to SI units [mG]->[m/s^2]
	Eigen::Vector3d readAcc();
	// Convert to SI units [mGauss]->[Tesla]
	Eigen::Vector3d readMag();

	Eigen::Vector3d computeGyrBias();
	Eigen::Vector3d computeAccBias();
	Eigen::Vector3d computeMagBias();

private:
	void selectDevice(int32_t file, int32_t addr);
	void readBytes(int32_t addr, uint8_t reg, uint8_t size, uint8_t *data);
	void readByte(int32_t addr, uint8_t reg, uint8_t *data);
	void writeByte(int32_t addr, uint8_t reg, uint8_t value);
};
