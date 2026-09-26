#pragma once

#include "lq_lsm6dsr.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace rewrite_path {

class Lsm6dsrSpi1 {
public:
    explicit Lsm6dsrSpi1(std::string device = "/dev/spidev1.0",
                         std::uint32_t speed_hz = 8000000);

    bool init(lsm6dsr_odr_t odr = LSM6DSR_ODR_104HZ,
              lsm6dsr_accel_fs_t accel_fs = LSM6DSR_ACCEL_FS_2G,
              lsm6dsr_gyro_fs_t gyro_fs = LSM6DSR_GYRO_FS_2000DPS);
    bool power_down();
    bool read_sample(lsm6dsr_sample_t& sample);
    const std::string& last_error() const;

private:
    bool reset();
    bool set_data_rate_and_scale(lsm6dsr_odr_t odr,
                                 lsm6dsr_accel_fs_t accel_fs,
                                 lsm6dsr_gyro_fs_t gyro_fs);
    bool read_reg(std::uint8_t reg, std::uint8_t& value);
    bool write_reg(std::uint8_t reg, std::uint8_t value);
    bool read_bytes(std::uint8_t reg, std::uint8_t* data, std::size_t length);
    static float accel_sensitivity_g(lsm6dsr_accel_fs_t fs);
    static float gyro_sensitivity_dps(lsm6dsr_gyro_fs_t fs);
    static std::int16_t make_i16(std::uint8_t low, std::uint8_t high);

    std::string device_;
    std::uint32_t speed_hz_;
    std::string error_;
    float accel_lsb_to_g_ = 0.000061f;
    float gyro_lsb_to_dps_ = 0.00875f;
};

}  // namespace rewrite_path
