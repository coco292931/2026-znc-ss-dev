/********************************************************************************
 * @file            lq_lsm6dsr.cpp
 * @brief           LSM6DSR 6-axis IMU driver implementation.
 ********************************************************************************/

#include "lq_lsm6dsr.hpp"
#include <cstdio>
#include <unistd.h>

lq_lsm6dsr::lq_lsm6dsr()
    : lq_lsm6dsr(LSM6DSR_DEFAULT_SCL_PIN, LSM6DSR_DEFAULT_SDA_PIN,
                 LSM6DSR_I2C_ADDR_7BIT)
{
}

lq_lsm6dsr::lq_lsm6dsr(gpio_pin_t scl_pin, gpio_pin_t sda_pin,
                       uint8_t addr_7bit)
    : i2c_(nullptr), addr_7bit_(addr_7bit), initialized_(false),
      odr_(LSM6DSR_ODR_POWER_DOWN), accel_fs_(LSM6DSR_ACCEL_FS_2G),
      gyro_fs_(LSM6DSR_GYRO_FS_250DPS), accel_lsb_to_g_(0.000061f),
      gyro_lsb_to_dps_(0.00875f)
{
    i2c_ = new lsm6dsr_i2c(scl_pin, sda_pin, addr_7bit_);
}

lq_lsm6dsr::~lq_lsm6dsr()
{
    delete i2c_;
    i2c_ = nullptr;
}

bool lq_lsm6dsr::write_reg(uint8_t reg, uint8_t value)
{
    return i2c_ && i2c_->write_byte(reg, value);
}

bool lq_lsm6dsr::read_reg(uint8_t reg, uint8_t &value)
{
    return i2c_ && i2c_->read_byte(reg, value);
}

bool lq_lsm6dsr::is_alive()
{
    uint8_t who = 0;
    return read_reg(LSM6DSR_REG_WHO_AM_I, who) && who == LSM6DSR_WHO_AM_I_VALUE;
}

bool lq_lsm6dsr::reset()
{
    if (!write_reg(LSM6DSR_REG_CTRL3_C, LSM6DSR_CTRL3_C_SW_RESET)) return false;

    for (int i = 0; i < 100; i++) {
        uint8_t v = LSM6DSR_CTRL3_C_SW_RESET;
        ::usleep(1000);
        if (read_reg(LSM6DSR_REG_CTRL3_C, v) &&
            (v & LSM6DSR_CTRL3_C_SW_RESET) == 0) {
            break;
        }
        if (i == 99) return false;
    }

    return write_reg(LSM6DSR_REG_CTRL3_C,
                     static_cast<uint8_t>(LSM6DSR_CTRL3_C_BDU |
                                          LSM6DSR_CTRL3_C_IF_INC));
}

bool lq_lsm6dsr::init(lsm6dsr_odr_t odr,
                      lsm6dsr_accel_fs_t accel_fs,
                      lsm6dsr_gyro_fs_t gyro_fs)
{
    if (!i2c_) {
        printf("[LSM6DSR] No I2C object\n");
        return false;
    }

    uint8_t who = 0;
    if (!read_reg(LSM6DSR_REG_WHO_AM_I, who)) {
        printf("[LSM6DSR] WHO_AM_I read failed. Check SCL/SDA wiring and pull-ups.\n");
        return false;
    }

    printf("[LSM6DSR] WHO_AM_I=0x%02X addr=0x%02X\n", who, addr_7bit_);
    if (who != LSM6DSR_WHO_AM_I_VALUE) {
        printf("[LSM6DSR] Device not found. Expected WHO_AM_I=0x%02X.\n",
               LSM6DSR_WHO_AM_I_VALUE);
        return false;
    }

    if (!reset()) {
        printf("[LSM6DSR] reset failed\n");
        return false;
    }

    if (!set_data_rate_and_scale(odr, accel_fs, gyro_fs)) {
        printf("[LSM6DSR] configure ODR/full-scale failed\n");
        return false;
    }

    initialized_ = true;
    return true;
}

bool lq_lsm6dsr::set_data_rate_and_scale(lsm6dsr_odr_t odr,
                                         lsm6dsr_accel_fs_t accel_fs,
                                         lsm6dsr_gyro_fs_t gyro_fs)
{
    uint8_t ctrl1_xl = static_cast<uint8_t>(odr) |
                       static_cast<uint8_t>(accel_fs);
    uint8_t ctrl2_g = static_cast<uint8_t>(odr) |
                      static_cast<uint8_t>(gyro_fs);

    if (!write_reg(LSM6DSR_REG_CTRL1_XL, ctrl1_xl)) return false;
    if (!write_reg(LSM6DSR_REG_CTRL2_G, ctrl2_g)) return false;

    odr_ = odr;
    accel_fs_ = accel_fs;
    gyro_fs_ = gyro_fs;
    accel_lsb_to_g_ = accel_sensitivity_g(accel_fs_);
    gyro_lsb_to_dps_ = gyro_sensitivity_dps(gyro_fs_);
    return true;
}

bool lq_lsm6dsr::power_down()
{
    bool ok = write_reg(LSM6DSR_REG_CTRL1_XL, 0x00) &&
              write_reg(LSM6DSR_REG_CTRL2_G, 0x00);
    if (ok) {
        initialized_ = false;
        odr_ = LSM6DSR_ODR_POWER_DOWN;
    }
    return ok;
}

bool lq_lsm6dsr::read_status(uint8_t &status)
{
    return read_reg(LSM6DSR_REG_STATUS_REG, status);
}

bool lq_lsm6dsr::accel_data_ready()
{
    uint8_t status = 0;
    return read_status(status) && ((status & 0x01) != 0);
}

bool lq_lsm6dsr::gyro_data_ready()
{
    uint8_t status = 0;
    return read_status(status) && ((status & 0x02) != 0);
}

bool lq_lsm6dsr::temp_data_ready()
{
    uint8_t status = 0;
    return read_status(status) && ((status & 0x04) != 0);
}

bool lq_lsm6dsr::read_raw(lsm6dsr_raw_t &raw)
{
    if (!i2c_) return false;

    uint8_t status = 0;
    if (!read_status(status)) return false;

    uint8_t buf[14] = {0};
    if (!i2c_->read_bytes(LSM6DSR_REG_OUT_TEMP_L, buf, sizeof(buf))) {
        return false;
    }

    raw.temperature = make_i16(buf[0], buf[1]);
    raw.gyro_x = make_i16(buf[2], buf[3]);
    raw.gyro_y = make_i16(buf[4], buf[5]);
    raw.gyro_z = make_i16(buf[6], buf[7]);
    raw.accel_x = make_i16(buf[8], buf[9]);
    raw.accel_y = make_i16(buf[10], buf[11]);
    raw.accel_z = make_i16(buf[12], buf[13]);
    raw.status = status;
    return true;
}

bool lq_lsm6dsr::read_sample(lsm6dsr_sample_t &sample)
{
    lsm6dsr_raw_t raw;
    if (!read_raw(raw)) return false;

    sample.accel_g[0] = static_cast<float>(raw.accel_x) * accel_lsb_to_g_;
    sample.accel_g[1] = static_cast<float>(raw.accel_y) * accel_lsb_to_g_;
    sample.accel_g[2] = static_cast<float>(raw.accel_z) * accel_lsb_to_g_;
    sample.gyro_dps[0] = static_cast<float>(raw.gyro_x) * gyro_lsb_to_dps_;
    sample.gyro_dps[1] = static_cast<float>(raw.gyro_y) * gyro_lsb_to_dps_;
    sample.gyro_dps[2] = static_cast<float>(raw.gyro_z) * gyro_lsb_to_dps_;
    sample.temperature_c = 25.0f + static_cast<float>(raw.temperature) / 256.0f;
    sample.status = raw.status;
    return true;
}

float lq_lsm6dsr::accel_sensitivity_g(lsm6dsr_accel_fs_t fs)
{
    switch (fs) {
    case LSM6DSR_ACCEL_FS_2G:  return 0.000061f;
    case LSM6DSR_ACCEL_FS_4G:  return 0.000122f;
    case LSM6DSR_ACCEL_FS_8G:  return 0.000244f;
    case LSM6DSR_ACCEL_FS_16G: return 0.000488f;
    default:                  return 0.000061f;
    }
}

float lq_lsm6dsr::gyro_sensitivity_dps(lsm6dsr_gyro_fs_t fs)
{
    switch (fs) {
    case LSM6DSR_GYRO_FS_125DPS:  return 0.004375f;
    case LSM6DSR_GYRO_FS_250DPS:  return 0.00875f;
    case LSM6DSR_GYRO_FS_500DPS:  return 0.01750f;
    case LSM6DSR_GYRO_FS_1000DPS: return 0.03500f;
    case LSM6DSR_GYRO_FS_2000DPS: return 0.07000f;
    case LSM6DSR_GYRO_FS_4000DPS: return 0.14000f;
    default:                     return 0.00875f;
    }
}

int16_t lq_lsm6dsr::make_i16(uint8_t lo, uint8_t hi)
{
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}
