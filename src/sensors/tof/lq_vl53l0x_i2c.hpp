/********************************************************************************
 * @file            lq_vl53l0x_i2c.hpp
 * @brief           VL53L0X software I2C helper with 8-bit register addresses.
 ********************************************************************************/

#ifndef __LQ_VL53L0X_I2C_HPP
#define __LQ_VL53L0X_I2C_HPP

#include "lq_lsm6dsr_i2c.hpp"
#include <cstdint>
#include <unistd.h>

#define VL53L0X_I2C_ADDR_7BIT (0x29)

class vl53l0x_i2c {
public:
    vl53l0x_i2c(gpio_pin_t scl_pin, gpio_pin_t sda_pin,
                uint8_t addr_7bit = VL53L0X_I2C_ADDR_7BIT)
        : scl_(scl_pin, GPIO_MODE_OUT), sda_(sda_pin, GPIO_MODE_OUT),
          addr_7bit_(addr_7bit)
    {
        release_sda();
        release_scl_and_wait();
        recover_bus();
    }

    bool write_byte(uint8_t reg, uint8_t value) {
        if (!start()) return false;
        if (!send_addr_write()) { stop(); return false; }
        if (!send_byte(reg)) { stop(); return false; }
        if (!wait_ack()) { stop(); return false; }
        if (!send_byte(value)) { stop(); return false; }
        if (!wait_ack()) { stop(); return false; }
        stop();
        return true;
    }

    bool write_word(uint8_t reg, uint16_t value) {
        uint8_t data[2] = {
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>(value & 0xFF),
        };
        return write_bytes(reg, data, 2);
    }

    bool write_bytes(uint8_t reg, const uint8_t *data, uint8_t len) {
        if (data == nullptr || len == 0) return false;
        if (!start()) return false;
        if (!send_addr_write()) { stop(); return false; }
        if (!send_byte(reg)) { stop(); return false; }
        if (!wait_ack()) { stop(); return false; }
        for (uint8_t i = 0; i < len; i++) {
            if (!send_byte(data[i])) { stop(); return false; }
            if (!wait_ack()) { stop(); return false; }
        }
        stop();
        return true;
    }

    bool read_byte(uint8_t reg, uint8_t &value) {
        if (!start()) return false;
        if (!send_addr_write()) { stop(); return false; }
        if (!send_byte(reg)) { stop(); return false; }
        if (!wait_ack()) { stop(); return false; }
        if (!start()) { stop(); return false; }
        if (!send_addr_read()) { stop(); return false; }
        if (!read_byte_internal(false, value)) {
            stop();
            return false;
        }
        stop();
        return true;
    }

    bool read_word(uint8_t reg, uint16_t &value) {
        uint8_t data[2] = {0, 0};
        if (!read_bytes(reg, data, 2)) return false;
        value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        return true;
    }

    bool read_bytes(uint8_t reg, uint8_t *data, uint8_t len) {
        if (data == nullptr || len == 0) return false;
        if (!start()) return false;
        if (!send_addr_write()) { stop(); return false; }
        if (!send_byte(reg)) { stop(); return false; }
        if (!wait_ack()) { stop(); return false; }
        if (!start()) { stop(); return false; }
        if (!send_addr_read()) { stop(); return false; }
        for (uint8_t i = 0; i < len; i++) {
            if (!read_byte_internal(i != (len - 1), data[i])) {
                stop();
                return false;
            }
        }
        stop();
        return true;
    }

private:
    static const uint16_t I2C_DELAY_US = 5;
    static const uint16_t CLOCK_STRETCH_TIMEOUT_US = 2000;

    void delay_us(uint16_t us) { ::usleep(us); }

    void drive_scl_low() {
        scl_.gpio_level_set(GPIO_LOW);
        scl_.gpio_direction_set(GPIO_MODE_OUT);
    }

    void release_scl() {
        scl_.gpio_direction_set(GPIO_MODE_IN);
    }

    void drive_sda_low() {
        sda_.gpio_level_set(GPIO_LOW);
        sda_.gpio_direction_set(GPIO_MODE_OUT);
    }

    void release_sda() {
        sda_.gpio_direction_set(GPIO_MODE_IN);
    }

    bool release_scl_and_wait() {
        release_scl();
        for (uint16_t elapsed = 0;
             elapsed < CLOCK_STRETCH_TIMEOUT_US;
             ++elapsed) {
            if (scl_.gpio_level_get() == GPIO_HIGH) return true;
            delay_us(1);
        }
        return false;
    }

    void recover_bus() {
        release_sda();
        for (int i = 0; i < 9; ++i) {
            drive_scl_low();
            delay_us(I2C_DELAY_US);
            if (!release_scl_and_wait()) break;
            delay_us(I2C_DELAY_US);
        }
        stop();
    }

    bool start() {
        drive_scl_low();
        release_sda();
        delay_us(I2C_DELAY_US);
        if (!release_scl_and_wait()) return false;
        delay_us(I2C_DELAY_US);
        drive_sda_low();
        delay_us(I2C_DELAY_US);
        drive_scl_low();
        delay_us(I2C_DELAY_US);
        return true;
    }

    void stop() {
        drive_scl_low();
        drive_sda_low();
        delay_us(I2C_DELAY_US);
        release_scl_and_wait();
        delay_us(I2C_DELAY_US);
        release_sda();
        delay_us(I2C_DELAY_US);
    }

    bool send_addr_write() {
        if (!send_byte(static_cast<uint8_t>(
                (addr_7bit_ << 1) | 0x00))) return false;
        return wait_ack();
    }

    bool send_addr_read() {
        if (!send_byte(static_cast<uint8_t>(
                (addr_7bit_ << 1) | 0x01))) return false;
        return wait_ack();
    }

    bool send_byte(uint8_t data) {
        drive_scl_low();
        for (int i = 7; i >= 0; i--) {
            if ((data & (1 << i)) != 0) {
                release_sda();
            } else {
                drive_sda_low();
            }
            delay_us(I2C_DELAY_US);
            if (!release_scl_and_wait()) return false;
            delay_us(I2C_DELAY_US);
            drive_scl_low();
            delay_us(I2C_DELAY_US);
        }
        return true;
    }

    bool wait_ack() {
        release_sda();
        delay_us(I2C_DELAY_US);
        if (!release_scl_and_wait()) return false;
        delay_us(I2C_DELAY_US);
        const bool acknowledged = sda_.gpio_level_get() == GPIO_LOW;
        drive_scl_low();
        delay_us(I2C_DELAY_US);
        return acknowledged;
    }

    bool send_ack() {
        drive_scl_low();
        drive_sda_low();
        delay_us(I2C_DELAY_US);
        if (!release_scl_and_wait()) return false;
        delay_us(I2C_DELAY_US);
        drive_scl_low();
        release_sda();
        return true;
    }

    bool send_nack() {
        drive_scl_low();
        release_sda();
        delay_us(I2C_DELAY_US);
        if (!release_scl_and_wait()) return false;
        delay_us(I2C_DELAY_US);
        drive_scl_low();
        return true;
    }

    bool read_byte_internal(bool send_ack_flag, uint8_t &data) {
        data = 0;
        release_sda();
        for (int i = 0; i < 8; i++) {
            drive_scl_low();
            delay_us(I2C_DELAY_US);
            if (!release_scl_and_wait()) return false;
            delay_us(I2C_DELAY_US);
            data = static_cast<uint8_t>((data << 1) |
                   (sda_.gpio_level_get() == GPIO_HIGH ? 1 : 0));
            drive_scl_low();
        }
        return send_ack_flag ? send_ack() : send_nack();
    }

private:
    ls_gpio scl_;
    ls_gpio sda_;
    uint8_t addr_7bit_;
};

#endif
