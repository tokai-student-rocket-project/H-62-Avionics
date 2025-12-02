#pragma once
#include "Lib_BNO055.hpp"

/**
 * @brief BNO055 Calibration Profile
 * 
 * Paste your calibration values here after running the calibration sketch.
 * Use this struct in your setup() to restore calibration.
 * 
 * Example:
 *   bno.setSensorOffsets(BNO055_CALIBRATION_PROFILE);
 */
const BNO055::bno055_offsets_t BNO055_CALIBRATION_PROFILE = {
    .accel_offset_x = -8,
    .accel_offset_y = 2,
    .accel_offset_z = -18,
    .mag_offset_x = -530,
    .mag_offset_y = -69,
    .mag_offset_z = 415,
    .gyro_offset_x = 2,
    .gyro_offset_y = -1,
    .gyro_offset_z = 1,
    .accel_radius = 1000,
    .mag_radius = 736
};
