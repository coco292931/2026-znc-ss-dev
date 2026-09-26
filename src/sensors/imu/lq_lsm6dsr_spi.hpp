/********************************************************************************
 * @file            lq_lsm6dsr_spi.hpp
 * @brief           LSM6DSR software-SPI helper and SPI IMU driver.
 *
 * Wiring default:
 *   MOSI/SDA -> PIN_62
 *   SCK/SCL  -> PIN_60
 *   MISO     -> PIN_61
 *   CS       -> PIN_25
 *   INT      -> ADC7 (optional status read)
 ********************************************************************************/

#ifndef __LQ_LSM6DSR_SPI_HPP
#define __LQ_LSM6DSR_SPI_HPP

#include "lq_lsm6dsr.hpp"
#include "LQ_HW_ADC.hpp"
#include <cstdint>
#include <unistd.h>

static const gpio_pin_t LSM6DSR_SPI_DEFAULT_MOSI_PIN = static_cast<gpio_pin_t>(62);
static const gpio_pin_t LSM6DSR_SPI_DEFAULT_SCK_PIN  = static_cast<gpio_pin_t>(60);
static const gpio_pin_t LSM6DSR_SPI_DEFAULT_MISO_PIN = static_cast<gpio_pin_t>(61);
static const gpio_pin_t LSM6DSR_SPI_DEFAULT_CS_PIN   = static_cast<gpio_pin_t>(25);
static const uint8_t    LSM6DSR_SPI_DEFAULT_INT_ADC  = 7;

#define LSM6DSR_SPI_READ_BIT            (0x80)

class lsm6dsr_spi {
public:
    lsm6dsr_spi(gpio_pin_t sck_pin = LSM6DSR_SPI_DEFAULT_SCK_PIN,
                gpio_pin_t mosi_pin = LSM6DSR_SPI_DEFAULT_MOSI_PIN,
                gpio_pin_t miso_pin = LSM6DSR_SPI_DEFAULT_MISO_PIN,
                gpio_pin_t cs_pin = LSM6DSR_SPI_DEFAULT_CS_PIN,
                uint16_t delay_us = 1)
        : sck_(sck_pin, GPIO_MODE_OUT), mosi_(mosi_pin, GPIO_MODE_OUT),
          miso_(miso_pin, GPIO_MODE_IN), cs_(cs_pin, GPIO_MODE_OUT),
          delay_us_(delay_us)
    {
        cs_.gpio_level_set(GPIO_HIGH);
        sck_.gpio_level_set(GPIO_LOW);
        mosi_.gpio_level_set(GPIO_LOW);
    }

    bool write_byte(uint8_t reg, uint8_t value) {
        select();
        transfer_byte(spi_write_addr(reg));
        transfer_byte(value);
        deselect();
        return true;
    }

    bool write_bytes(uint8_t reg, const uint8_t *data, uint8_t len) {
        if (data == nullptr || len == 0) return false;
        select();
        transfer_byte(spi_write_addr(reg));
        for (uint8_t i = 0; i < len; i++) {
            transfer_byte(data[i]);
        }
        deselect();
        return true;
    }

    bool read_byte(uint8_t reg, uint8_t &value) {
        select();
        transfer_byte(spi_read_addr(reg));
        value = transfer_byte(0xFF);
        deselect();
        return true;
    }

    bool read_bytes(uint8_t reg, uint8_t *data, uint8_t len) {
        if (data == nullptr || len == 0) return false;
        select();
        transfer_byte(spi_read_addr(reg));
        for (uint8_t i = 0; i < len; i++) {
            data[i] = transfer_byte(0xFF);
        }
        deselect();
        return true;
    }

    bool update_bits(uint8_t reg, uint8_t mask, uint8_t value) {
        uint8_t current = 0;
        if (!read_byte(reg, current)) return false;
        current = static_cast<uint8_t>((current & ~mask) | (value & mask));
        return write_byte(reg, current);
    }

private:
    static uint8_t spi_read_addr(uint8_t reg) {
        return static_cast<uint8_t>((reg & 0x7F) | LSM6DSR_SPI_READ_BIT);
    }

    static uint8_t spi_write_addr(uint8_t reg) {
        return static_cast<uint8_t>(reg & 0x7F);
    }

    void delay() {
        if (delay_us_ > 0) ::usleep(delay_us_);
    }

    void select() {
        sck_.gpio_level_set(GPIO_LOW);
        delay();
        cs_.gpio_level_set(GPIO_LOW);
        delay();
    }

    void deselect() {
        delay();
        sck_.gpio_level_set(GPIO_LOW);
        cs_.gpio_level_set(GPIO_HIGH);
        delay();
    }

    uint8_t transfer_byte(uint8_t tx) {
        uint8_t rx = 0;
        for (int bit = 7; bit >= 0; bit--) {
            mosi_.gpio_level_set((tx & (1 << bit)) ? GPIO_HIGH : GPIO_LOW);
            delay();
            sck_.gpio_level_set(GPIO_HIGH);
            delay();
            rx = static_cast<uint8_t>((rx << 1) |
                 (miso_.gpio_level_get() == GPIO_HIGH ? 1 : 0));
            sck_.gpio_level_set(GPIO_LOW);
            delay();
        }
        return rx;
    }

private:
    ls_gpio sck_;
    ls_gpio mosi_;
    ls_gpio miso_;
    ls_gpio cs_;
    uint16_t delay_us_;
};

class lq_lsm6dsr_spi {
public:
    lq_lsm6dsr_spi();
    lq_lsm6dsr_spi(gpio_pin_t sck_pin, gpio_pin_t mosi_pin,
                   gpio_pin_t miso_pin, gpio_pin_t cs_pin,
                   uint8_t int_adc_channel = LSM6DSR_SPI_DEFAULT_INT_ADC);
    ~lq_lsm6dsr_spi();

    lq_lsm6dsr_spi(const lq_lsm6dsr_spi&) = delete;
    lq_lsm6dsr_spi& operator=(const lq_lsm6dsr_spi&) = delete;

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

    int read_int_raw();
    float read_int_voltage();

private:
    static float accel_sensitivity_g(lsm6dsr_accel_fs_t fs);
    static float gyro_sensitivity_dps(lsm6dsr_gyro_fs_t fs);
    static int16_t make_i16(uint8_t lo, uint8_t hi);

private:
    lsm6dsr_spi *spi_;
    HWAdc *int_adc_;
    gpio_pin_t sck_pin_;
    gpio_pin_t mosi_pin_;
    gpio_pin_t miso_pin_;
    gpio_pin_t cs_pin_;
    uint8_t int_adc_channel_;
    bool initialized_;
    lsm6dsr_odr_t odr_;
    lsm6dsr_accel_fs_t accel_fs_;
    lsm6dsr_gyro_fs_t gyro_fs_;
    float accel_lsb_to_g_;
    float gyro_lsb_to_dps_;
};

#endif
