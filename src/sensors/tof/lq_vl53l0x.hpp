/********************************************************************************
 * @file            lq_vl53l0x.hpp
 * @brief           VL53L0X time-of-flight ranging driver.
 * @description     Ported from the provided MicroPython VL53L0X driver and
 *                  adapted to the existing Loongson GPIO software-I2C style.
 ********************************************************************************/

#ifndef __LQ_VL53L0X_HPP
#define __LQ_VL53L0X_HPP

#include "lq_vl53l0x_i2c.hpp"
#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>

#define VL53L0X_REG_SYSRANGE_START                         (0x00)
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG                 (0x01)
#define VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD         (0x04)
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO           (0x0A)
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                 (0x0B)
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS                (0x13)
#define VL53L0X_REG_RESULT_RANGE_STATUS                    (0x14)
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN  (0x44)
#define VL53L0X_REG_MSRC_CONFIG_CONTROL                    (0x60)
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                (0x84)
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV       (0x89)
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0       (0xB0)
#define VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT      (0xB6)
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID                (0xC0)
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID             (0xC2)
#define VL53L0X_REG_OSC_CALIBRATE_VAL                      (0xF8)

typedef struct {
    bool     valid;
    uint16_t distance_mm;
    uint8_t  range_status;
    uint8_t  interrupt_status;
    const char *status_text;
} vl53l0x_result_t;

typedef std::function<void(const vl53l0x_result_t&)> vl53l0x_callback_t;

class lq_vl53l0x {
public:
    lq_vl53l0x();
    lq_vl53l0x(gpio_pin_t scl_pin, gpio_pin_t sda_pin,
               gpio_pin_t xshut_pin = static_cast<gpio_pin_t>(0xFF),
               uint8_t addr_7bit = VL53L0X_I2C_ADDR_7BIT);
    ~lq_vl53l0x();

    lq_vl53l0x(const lq_vl53l0x&) = delete;
    lq_vl53l0x& operator=(const lq_vl53l0x&) = delete;

    bool init(bool power_2v8 = true);
    bool is_initialized() const { return initialized_; }
    bool is_alive();
    bool get_model_info(uint8_t &model_id, uint8_t &revision_id);

    bool read_result(vl53l0x_result_t &result, bool blocking = true,
                     uint32_t timeout_ms = 1000);
    uint16_t get_distance_mm(bool blocking = true, uint32_t timeout_ms = 1000);

    bool start_continuous(uint16_t period_ms = 0);
    void stop_continuous();
    bool is_measuring() const { return measuring_.load(); }

    bool data_ready();
    bool wait_data_ready(uint32_t timeout_ms = 1000);
    void clear_interrupt();

    bool set_signal_rate_limit(float limit_mcps);
    void hardware_reset();
    void set_xshut(bool level);
    void set_callback(vl53l0x_callback_t cb) { callback_ = cb; }

private:
    bool write_reg(uint8_t reg, uint8_t value);
    bool write_reg16(uint8_t reg, uint16_t value);
    bool read_reg(uint8_t reg, uint8_t &value);
    bool read_reg16(uint8_t reg, uint16_t &value);
    bool read_regs(uint8_t reg, uint8_t *data, uint8_t len);
    bool set_flag(uint8_t reg, uint8_t bit, bool value);
    bool get_spad_info(uint8_t &count, bool &is_aperture);
    bool calibrate(uint8_t vhv_init_byte);
    bool write_static_init_sequence();
    bool prepare_single_shot();
    static const char* status_text(uint8_t range_status, uint16_t distance_mm);

private:
    vl53l0x_i2c *i2c_;
    ls_gpio     *xshut_pin_;
    uint8_t      addr_7bit_;
    uint8_t      stop_variable_;
    bool         initialized_;
    bool         started_;

    std::thread          *measure_thread_;
    std::atomic<bool>     measuring_;
    std::atomic<uint16_t> measure_period_ms_;
    vl53l0x_callback_t    callback_;
};

#endif
