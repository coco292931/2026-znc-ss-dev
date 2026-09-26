#include "lsm6dsr_spi1.hpp"

#include "spi1_shared.hpp"

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <utility>
#include <vector>

namespace rewrite_path {

Lsm6dsrSpi1::Lsm6dsrSpi1(std::string device, std::uint32_t speed_hz)
    : device_(std::move(device)), speed_hz_(speed_hz) {}

bool Lsm6dsrSpi1::write_reg(std::uint8_t reg, std::uint8_t value) {
    const std::uint8_t tx[] = {
        static_cast<std::uint8_t>(reg & 0x7f), value,
    };
    if (!spi1_shared_bus().transfer(
            Spi1Chip::Imu, tx, nullptr, sizeof(tx), speed_hz_)) {
        error_ = spi1_shared_bus().last_error();
        return false;
    }
    return true;
}

bool Lsm6dsrSpi1::read_reg(std::uint8_t reg, std::uint8_t& value) {
    return read_bytes(reg, &value, 1);
}

bool Lsm6dsrSpi1::read_bytes(std::uint8_t reg,
                             std::uint8_t* data,
                             std::size_t length) {
    if (!data || length == 0) {
        error_ = "Invalid LSM6DSR read buffer";
        return false;
    }
    std::vector<std::uint8_t> tx(length + 1, 0xff);
    std::vector<std::uint8_t> rx(length + 1, 0);
    tx[0] = static_cast<std::uint8_t>((reg & 0x7f) | 0x80);
    if (!spi1_shared_bus().transfer(
            Spi1Chip::Imu, tx.data(), rx.data(), tx.size(), speed_hz_)) {
        error_ = spi1_shared_bus().last_error();
        return false;
    }
    std::memcpy(data, rx.data() + 1, length);
    return true;
}

bool Lsm6dsrSpi1::reset() {
    if (!write_reg(LSM6DSR_REG_CTRL3_C, LSM6DSR_CTRL3_C_SW_RESET)) {
        return false;
    }
    for (int attempt = 0; attempt < 100; ++attempt) {
        ::usleep(1000);
        std::uint8_t value = LSM6DSR_CTRL3_C_SW_RESET;
        if (read_reg(LSM6DSR_REG_CTRL3_C, value) &&
            (value & LSM6DSR_CTRL3_C_SW_RESET) == 0) {
            return write_reg(
                LSM6DSR_REG_CTRL3_C,
                static_cast<std::uint8_t>(LSM6DSR_CTRL3_C_BDU |
                                          LSM6DSR_CTRL3_C_IF_INC));
        }
    }
    error_ = "LSM6DSR reset timed out";
    return false;
}

bool Lsm6dsrSpi1::set_data_rate_and_scale(
    lsm6dsr_odr_t odr,
    lsm6dsr_accel_fs_t accel_fs,
    lsm6dsr_gyro_fs_t gyro_fs) {
    const std::uint8_t ctrl1_xl =
        static_cast<std::uint8_t>(odr) | static_cast<std::uint8_t>(accel_fs);
    const std::uint8_t ctrl2_g =
        static_cast<std::uint8_t>(odr) | static_cast<std::uint8_t>(gyro_fs);
    if (!write_reg(LSM6DSR_REG_CTRL1_XL, ctrl1_xl) ||
        !write_reg(LSM6DSR_REG_CTRL2_G, ctrl2_g)) {
        return false;
    }
    accel_lsb_to_g_ = accel_sensitivity_g(accel_fs);
    gyro_lsb_to_dps_ = gyro_sensitivity_dps(gyro_fs);
    return true;
}

bool Lsm6dsrSpi1::init(lsm6dsr_odr_t odr,
                       lsm6dsr_accel_fs_t accel_fs,
                       lsm6dsr_gyro_fs_t gyro_fs) {
    if (!spi1_shared_bus().open(device_)) {
        error_ = spi1_shared_bus().last_error();
        return false;
    }
    std::uint8_t who = 0;
    if (!read_reg(LSM6DSR_REG_WHO_AM_I, who)) return false;
    std::printf("[LSM6DSR-SPI1] WHO_AM_I=0x%02X CS=25 speed=%uHz\n",
                who, static_cast<unsigned>(speed_hz_));
    if (who != LSM6DSR_WHO_AM_I_VALUE) {
        char message[64];
        std::snprintf(message, sizeof(message),
                      "Unexpected LSM6DSR WHO_AM_I=0x%02X", who);
        error_ = message;
        return false;
    }
    return reset() && set_data_rate_and_scale(odr, accel_fs, gyro_fs);
}

bool Lsm6dsrSpi1::power_down() {
    return write_reg(LSM6DSR_REG_CTRL1_XL, 0) &&
           write_reg(LSM6DSR_REG_CTRL2_G, 0);
}

bool Lsm6dsrSpi1::read_sample(lsm6dsr_sample_t& sample) {
    std::uint8_t status = 0;
    if (!read_reg(LSM6DSR_REG_STATUS_REG, status)) return false;

    std::uint8_t data[14] = {};
    if (!read_bytes(LSM6DSR_REG_OUT_TEMP_L, data, sizeof(data))) return false;

    const std::int16_t temperature = make_i16(data[0], data[1]);
    sample.gyro_dps[0] = make_i16(data[2], data[3]) * gyro_lsb_to_dps_;
    sample.gyro_dps[1] = make_i16(data[4], data[5]) * gyro_lsb_to_dps_;
    sample.gyro_dps[2] = make_i16(data[6], data[7]) * gyro_lsb_to_dps_;
    sample.accel_g[0] = make_i16(data[8], data[9]) * accel_lsb_to_g_;
    sample.accel_g[1] = make_i16(data[10], data[11]) * accel_lsb_to_g_;
    sample.accel_g[2] = make_i16(data[12], data[13]) * accel_lsb_to_g_;
    sample.temperature_c = 25.0f + temperature / 256.0f;
    sample.status = status;
    return true;
}

const std::string& Lsm6dsrSpi1::last_error() const {
    return error_;
}

float Lsm6dsrSpi1::accel_sensitivity_g(lsm6dsr_accel_fs_t fs) {
    switch (fs) {
        case LSM6DSR_ACCEL_FS_2G:  return 0.000061f;
        case LSM6DSR_ACCEL_FS_4G:  return 0.000122f;
        case LSM6DSR_ACCEL_FS_8G:  return 0.000244f;
        case LSM6DSR_ACCEL_FS_16G: return 0.000488f;
        default:                   return 0.000061f;
    }
}

float Lsm6dsrSpi1::gyro_sensitivity_dps(lsm6dsr_gyro_fs_t fs) {
    switch (fs) {
        case LSM6DSR_GYRO_FS_125DPS:  return 0.004375f;
        case LSM6DSR_GYRO_FS_250DPS:  return 0.00875f;
        case LSM6DSR_GYRO_FS_500DPS:  return 0.01750f;
        case LSM6DSR_GYRO_FS_1000DPS: return 0.03500f;
        case LSM6DSR_GYRO_FS_2000DPS: return 0.07000f;
        case LSM6DSR_GYRO_FS_4000DPS: return 0.14000f;
        default:                      return 0.00875f;
    }
}

std::int16_t Lsm6dsrSpi1::make_i16(std::uint8_t low, std::uint8_t high) {
    return static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(high) << 8) | low);
}

}  // namespace rewrite_path
