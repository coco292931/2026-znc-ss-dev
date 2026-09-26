/********************************************************************************
 * @file            lq_lsm6dsr.hpp
 * @brief           LSM6DSR 6-axis IMU driver.
 ********************************************************************************/

#ifndef __LQ_LSM6DSR_HPP
#define __LQ_LSM6DSR_HPP

#include "lq_lsm6dsr_i2c.hpp"
#include <cstdint>

static const gpio_pin_t LSM6DSR_DEFAULT_SCL_PIN = static_cast<gpio_pin_t>(84); // TIM1_CH1N_84
static const gpio_pin_t LSM6DSR_DEFAULT_SDA_PIN = static_cast<gpio_pin_t>(85); // TIM1_CH2N_85

#define LSM6DSR_REG_FUNC_CFG_ACCESS       (0x01)
#define LSM6DSR_REG_WHO_AM_I              (0x0F)
#define LSM6DSR_REG_CTRL1_XL              (0x10)
#define LSM6DSR_REG_CTRL2_G               (0x11)
#define LSM6DSR_REG_CTRL3_C               (0x12)
#define LSM6DSR_REG_STATUS_REG            (0x1E)
#define LSM6DSR_REG_OUT_TEMP_L            (0x20)
#define LSM6DSR_REG_OUTX_L_G              (0x22)
#define LSM6DSR_REG_OUTX_L_A              (0x28)

#define LSM6DSR_WHO_AM_I_VALUE            (0x6B)
#define LSM6DSR_CTRL3_C_BDU               (0x40)
#define LSM6DSR_CTRL3_C_IF_INC            (0x04)
#define LSM6DSR_CTRL3_C_SW_RESET          (0x01)

typedef enum {
    LSM6DSR_ODR_POWER_DOWN = 0x00,
    LSM6DSR_ODR_12HZ5      = 0x10,
    LSM6DSR_ODR_26HZ       = 0x20,
    LSM6DSR_ODR_52HZ       = 0x30,
    LSM6DSR_ODR_104HZ      = 0x40,
    LSM6DSR_ODR_208HZ      = 0x50,
    LSM6DSR_ODR_416HZ      = 0x60,
    LSM6DSR_ODR_833HZ      = 0x70,
    LSM6DSR_ODR_1666HZ     = 0x80,
    LSM6DSR_ODR_3332HZ     = 0x90,
    LSM6DSR_ODR_6667HZ     = 0xA0,
} lsm6dsr_odr_t;

typedef enum {
    LSM6DSR_ACCEL_FS_2G  = 0x00,
    LSM6DSR_ACCEL_FS_16G = 0x04,
    LSM6DSR_ACCEL_FS_4G  = 0x08,
    LSM6DSR_ACCEL_FS_8G  = 0x0C,
} lsm6dsr_accel_fs_t;

typedef enum {
    LSM6DSR_GYRO_FS_250DPS  = 0x00,
    LSM6DSR_GYRO_FS_4000DPS = 0x01,
    LSM6DSR_GYRO_FS_125DPS  = 0x02,
    LSM6DSR_GYRO_FS_500DPS  = 0x04,
    LSM6DSR_GYRO_FS_1000DPS = 0x08,
    LSM6DSR_GYRO_FS_2000DPS = 0x0C,
} lsm6dsr_gyro_fs_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
    uint8_t status;
} lsm6dsr_raw_t;

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float temperature_c;
    uint8_t status;
} lsm6dsr_sample_t;

class lq_lsm6dsr {
public:
    lq_lsm6dsr();
    lq_lsm6dsr(gpio_pin_t scl_pin, gpio_pin_t sda_pin,
               uint8_t addr_7bit = LSM6DSR_I2C_ADDR_7BIT);
    ~lq_lsm6dsr();

    lq_lsm6dsr(const lq_lsm6dsr&) = delete;
    lq_lsm6dsr& operator=(const lq_lsm6dsr&) = delete;

    bool init(lsm6dsr_odr_t odr = LSM6DSR_ODR_104HZ,
              lsm6dsr_accel_fs_t accel_fs = LSM6DSR_ACCEL_FS_2G,
              lsm6dsr_gyro_fs_t gyro_fs = LSM6DSR_GYRO_FS_2000DPS);
    bool is_initialized() const { return initialized_; }
    bool is_alive();
    bool reset();
    bool power_down();

    bool set_data_rate_and_scale(lsm6dsr_odr_t odr,
                                 lsm6dsr_accel_fs_t accel_fs,
                                 lsm6dsr_gyro_fs_t gyro_fs);

    bool read_status(uint8_t &status);
    bool accel_data_ready();
    bool gyro_data_ready();
    bool temp_data_ready();

    bool read_raw(lsm6dsr_raw_t &raw);
    bool read_sample(lsm6dsr_sample_t &sample);

    bool read_reg(uint8_t reg, uint8_t &value);
    bool write_reg(uint8_t reg, uint8_t value);

private:
    static float accel_sensitivity_g(lsm6dsr_accel_fs_t fs);
    static float gyro_sensitivity_dps(lsm6dsr_gyro_fs_t fs);
    static int16_t make_i16(uint8_t lo, uint8_t hi);

private:
    lsm6dsr_i2c *i2c_;
    uint8_t addr_7bit_;
    bool initialized_;
    lsm6dsr_odr_t odr_;
    lsm6dsr_accel_fs_t accel_fs_;
    lsm6dsr_gyro_fs_t gyro_fs_;
    float accel_lsb_to_g_;
    float gyro_lsb_to_dps_;
};

#endif
