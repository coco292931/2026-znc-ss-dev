/********************************************************************************
 * @file            lq_lsm6dsr_i2c.hpp
 * @brief           LSM6DSR software I2C helper with 8-bit register addresses.
 ********************************************************************************/

#ifndef __LQ_LSM6DSR_I2C_HPP
#define __LQ_LSM6DSR_I2C_HPP

#include "LQ_HW_GPIO.hpp"
#include <cstdint>
#include <unistd.h>

using gpio_pin_t = uint8_t;

static const uint8_t GPIO_MODE_OUT = GPIO_Mode_Out;
static const uint8_t GPIO_MODE_IN = GPIO_Mode_In;
static const uint8_t GPIO_LOW = 0;
static const uint8_t GPIO_HIGH = 1;

class ls_gpio {
public:
    ls_gpio(gpio_pin_t pin, uint8_t mode) : gpio_(pin, mode) {}

    void gpio_direction_set(uint8_t mode) { gpio_.GpioDirection(mode); }
    void gpio_level_set(uint8_t value) { gpio_.SetGpioValue(value); }
    uint8_t gpio_level_get() { return gpio_.GetGpioValue() ? GPIO_HIGH : GPIO_LOW; }

private:
    HWGpio gpio_;
};

#define LSM6DSR_I2C_ADDR_7BIT_SDO_LOW  (0x6A)
#define LSM6DSR_I2C_ADDR_7BIT_SDO_HIGH (0x6B)
#define LSM6DSR_I2C_ADDR_7BIT          LSM6DSR_I2C_ADDR_7BIT_SDO_LOW

class lsm6dsr_i2c {
public:
    lsm6dsr_i2c(gpio_pin_t scl_pin, gpio_pin_t sda_pin,
                uint8_t addr_7bit = LSM6DSR_I2C_ADDR_7BIT)
        : scl_(scl_pin, GPIO_MODE_OUT), sda_(sda_pin, GPIO_MODE_OUT),
          addr_7bit_(addr_7bit)
    {
        sda_.gpio_level_set(GPIO_HIGH);
        scl_.gpio_level_set(GPIO_HIGH);
    }

    bool write_byte(uint8_t reg, uint8_t value) {
        start();
        if (!send_addr_write()) { stop(); return false; }
        send_byte(reg);
        if (!wait_ack()) { stop(); return false; }
        send_byte(value);
        if (!wait_ack()) { stop(); return false; }
        stop();
        return true;
    }

    bool write_bytes(uint8_t reg, const uint8_t *data, uint8_t len) {
        if (data == nullptr || len == 0) return false;
        start();
        if (!send_addr_write()) { stop(); return false; }
        send_byte(reg);
        if (!wait_ack()) { stop(); return false; }
        for (uint8_t i = 0; i < len; i++) {
            send_byte(data[i]);
            if (!wait_ack()) { stop(); return false; }
        }
        stop();
        return true;
    }

    bool read_byte(uint8_t reg, uint8_t &value) {
        start();
        if (!send_addr_write()) { stop(); return false; }
        send_byte(reg);
        if (!wait_ack()) { stop(); return false; }
        start();
        if (!send_addr_read()) { stop(); return false; }
        value = read_byte_internal(false);
        stop();
        return true;
    }

    bool read_bytes(uint8_t reg, uint8_t *data, uint8_t len) {
        if (data == nullptr || len == 0) return false;
        start();
        if (!send_addr_write()) { stop(); return false; }
        send_byte(reg);
        if (!wait_ack()) { stop(); return false; }
        start();
        if (!send_addr_read()) { stop(); return false; }
        for (uint8_t i = 0; i < len; i++) {
            data[i] = read_byte_internal(i != (len - 1));
        }
        stop();
        return true;
    }

    bool update_bits(uint8_t reg, uint8_t mask, uint8_t value) {
        uint8_t current = 0;
        if (!read_byte(reg, current)) return false;
        current = static_cast<uint8_t>((current & ~mask) | (value & mask));
        return write_byte(reg, current);
    }

private:
    static const uint16_t I2C_DELAY_US = 5;

    void delay_us(uint16_t us) { ::usleep(us); }

    void start() {
        sda_.gpio_direction_set(GPIO_MODE_OUT);
        sda_.gpio_level_set(GPIO_HIGH);
        scl_.gpio_level_set(GPIO_HIGH);
        delay_us(I2C_DELAY_US);
        sda_.gpio_level_set(GPIO_LOW);
        delay_us(I2C_DELAY_US);
        scl_.gpio_level_set(GPIO_LOW);
        delay_us(I2C_DELAY_US);
    }

    void stop() {
        sda_.gpio_direction_set(GPIO_MODE_OUT);
        sda_.gpio_level_set(GPIO_LOW);
        delay_us(I2C_DELAY_US);
        scl_.gpio_level_set(GPIO_HIGH);
        delay_us(I2C_DELAY_US);
        sda_.gpio_level_set(GPIO_HIGH);
        delay_us(I2C_DELAY_US);
    }

    bool send_addr_write() {
        send_byte(static_cast<uint8_t>((addr_7bit_ << 1) | 0x00));
        return wait_ack();
    }

    bool send_addr_read() {
        send_byte(static_cast<uint8_t>((addr_7bit_ << 1) | 0x01));
        return wait_ack();
    }

    void send_byte(uint8_t data) {
        sda_.gpio_direction_set(GPIO_MODE_OUT);
        scl_.gpio_level_set(GPIO_LOW);
        for (int i = 7; i >= 0; i--) {
            sda_.gpio_level_set((data & (1 << i)) ? GPIO_HIGH : GPIO_LOW);
            delay_us(I2C_DELAY_US);
            scl_.gpio_level_set(GPIO_HIGH);
            delay_us(I2C_DELAY_US);
            scl_.gpio_level_set(GPIO_LOW);
            delay_us(I2C_DELAY_US);
        }
    }

    bool wait_ack() {
        uint16_t err = 0;
        sda_.gpio_direction_set(GPIO_MODE_IN);
        delay_us(I2C_DELAY_US);
        scl_.gpio_level_set(GPIO_HIGH);
        delay_us(I2C_DELAY_US);
        while (sda_.gpio_level_get() == GPIO_HIGH) {
            if (++err > 2000) {
                scl_.gpio_level_set(GPIO_LOW);
                return false;
            }
            delay_us(I2C_DELAY_US);
        }
        scl_.gpio_level_set(GPIO_LOW);
        return true;
    }

    void send_ack() {
        sda_.gpio_direction_set(GPIO_MODE_OUT);
        sda_.gpio_level_set(GPIO_LOW);
        delay_us(1);
        scl_.gpio_level_set(GPIO_HIGH);
        delay_us(2);
        scl_.gpio_level_set(GPIO_LOW);
        delay_us(1);
    }

    void send_nack() {
        sda_.gpio_direction_set(GPIO_MODE_OUT);
        sda_.gpio_level_set(GPIO_HIGH);
        delay_us(1);
        scl_.gpio_level_set(GPIO_HIGH);
        delay_us(2);
        scl_.gpio_level_set(GPIO_LOW);
        delay_us(1);
    }

    uint8_t read_byte_internal(bool send_ack_flag) {
        uint8_t data = 0;
        sda_.gpio_direction_set(GPIO_MODE_IN);
        for (int i = 0; i < 8; i++) {
            scl_.gpio_level_set(GPIO_LOW);
            delay_us(1);
            scl_.gpio_level_set(GPIO_HIGH);
            data = static_cast<uint8_t>((data << 1) |
                   (sda_.gpio_level_get() == GPIO_HIGH ? 1 : 0));
            delay_us(1);
        }
        scl_.gpio_level_set(GPIO_LOW);
        if (send_ack_flag) send_ack(); else send_nack();
        return data;
    }

private:
    ls_gpio scl_;
    ls_gpio sda_;
    uint8_t addr_7bit_;
};

#endif
