/********************************************************************************
 * @file            lq_vl53l0x.cpp
 * @brief           VL53L0X time-of-flight ranging driver implementation.
 ********************************************************************************/

#include "lq_vl53l0x.hpp"
#include <cstdio>
#include <cstring>
#include <chrono>

static const uint32_t LQ_VL53L0X_IO_TIMEOUT_MS = 1000;

struct lq_vl53l0x_reg_value_t {
    uint8_t reg;
    uint8_t value;
};

static const lq_vl53l0x_reg_value_t LQ_VL53L0X_STATIC_INIT[] = {
    {0xFF,0x01},{0x00,0x00},
    {0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},
    {0xFF,0x01},{0x4E,0x2C},{0x48,0x00},{0x30,0x20},
    {0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},
    {0x51,0x00},{0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},
    {0x62,0x00},{0x64,0x00},{0x65,0x00},{0x66,0xA0},
    {0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},{0x4A,0x00},
    {0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},
    {0xFF,0x01},{0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},
    {0x46,0x05},{0x40,0x40},{0x0E,0x06},{0x20,0x1A},{0x43,0x40},
    {0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},
    {0xFF,0x00},{0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},
    {0x67,0x00},{0x70,0x04},{0x71,0x01},{0x72,0xFE},{0x76,0x00},
    {0x77,0x00},
    {0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},
    {0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00},
};

lq_vl53l0x::lq_vl53l0x()
    : i2c_(nullptr), xshut_pin_(nullptr), addr_7bit_(VL53L0X_I2C_ADDR_7BIT),
      stop_variable_(0), initialized_(false), started_(false),
      measure_thread_(nullptr), measuring_(false), measure_period_ms_(100),
      callback_(nullptr) {}

lq_vl53l0x::lq_vl53l0x(gpio_pin_t scl_pin, gpio_pin_t sda_pin,
                       gpio_pin_t xshut_pin, uint8_t addr_7bit)
    : i2c_(nullptr), xshut_pin_(nullptr), addr_7bit_(addr_7bit),
      stop_variable_(0), initialized_(false), started_(false),
      measure_thread_(nullptr), measuring_(false), measure_period_ms_(100),
      callback_(nullptr)
{
    i2c_ = new vl53l0x_i2c(scl_pin, sda_pin, addr_7bit_);
    if (xshut_pin != static_cast<gpio_pin_t>(0xFF)) {
        xshut_pin_ = new ls_gpio(xshut_pin, GPIO_MODE_OUT);
        xshut_pin_->gpio_level_set(GPIO_HIGH);
    }
}

lq_vl53l0x::~lq_vl53l0x()
{
    stop_continuous();
    delete i2c_;       i2c_ = nullptr;
    delete xshut_pin_; xshut_pin_ = nullptr;
}

bool lq_vl53l0x::write_reg(uint8_t reg, uint8_t value)
{
    return i2c_ && i2c_->write_byte(reg, value);
}

bool lq_vl53l0x::write_reg16(uint8_t reg, uint16_t value)
{
    return i2c_ && i2c_->write_word(reg, value);
}

bool lq_vl53l0x::read_reg(uint8_t reg, uint8_t &value)
{
    return i2c_ && i2c_->read_byte(reg, value);
}

bool lq_vl53l0x::read_reg16(uint8_t reg, uint16_t &value)
{
    return i2c_ && i2c_->read_word(reg, value);
}

bool lq_vl53l0x::read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    return i2c_ && i2c_->read_bytes(reg, data, len);
}

bool lq_vl53l0x::set_flag(uint8_t reg, uint8_t bit, bool value)
{
    uint8_t data = 0;
    if (!read_reg(reg, data)) return false;
    if (value) data |= static_cast<uint8_t>(1u << bit);
    else data &= static_cast<uint8_t>(~(1u << bit));
    return write_reg(reg, data);
}

void lq_vl53l0x::hardware_reset()
{
    if (!xshut_pin_) return;
    xshut_pin_->gpio_level_set(GPIO_LOW);
    usleep(50000);
    xshut_pin_->gpio_level_set(GPIO_HIGH);
    usleep(50000);
}

void lq_vl53l0x::set_xshut(bool level)
{
    if (xshut_pin_) xshut_pin_->gpio_level_set(level ? GPIO_HIGH : GPIO_LOW);
}

bool lq_vl53l0x::is_alive()
{
    uint8_t model = 0;
    return read_reg(VL53L0X_REG_IDENTIFICATION_MODEL_ID, model) && model == 0xEE;
}

bool lq_vl53l0x::get_model_info(uint8_t &model_id, uint8_t &revision_id)
{
    return read_reg(VL53L0X_REG_IDENTIFICATION_MODEL_ID, model_id) &&
           read_reg(VL53L0X_REG_IDENTIFICATION_REVISION_ID, revision_id);
}

bool lq_vl53l0x::get_spad_info(uint8_t &count, bool &is_aperture)
{
    if (!write_reg(0x80, 0x01) || !write_reg(0xFF, 0x01) ||
        !write_reg(0x00, 0x00) || !write_reg(0xFF, 0x06)) return false;
    if (!set_flag(0x83, 2, true)) return false;
    if (!write_reg(0xFF, 0x07) || !write_reg(0x81, 0x01) ||
        !write_reg(0x80, 0x01) || !write_reg(0x94, 0x6B) ||
        !write_reg(0x83, 0x00)) return false;

    uint8_t value = 0;
    uint32_t elapsed = 0;
    while (elapsed < LQ_VL53L0X_IO_TIMEOUT_MS) {
        if (!read_reg(0x83, value)) return false;
        if (value != 0) break;
        usleep(1000);
        elapsed++;
    }
    if (elapsed >= LQ_VL53L0X_IO_TIMEOUT_MS) return false;

    if (!write_reg(0x83, 0x01) || !read_reg(0x92, value)) return false;
    if (!write_reg(0x81, 0x00) || !write_reg(0xFF, 0x06)) return false;
    if (!set_flag(0x83, 2, false)) return false;
    if (!write_reg(0xFF, 0x01) || !write_reg(0x00, 0x01) ||
        !write_reg(0xFF, 0x00) || !write_reg(0x80, 0x00)) return false;

    count = value & 0x7F;
    is_aperture = (value & 0x80) != 0;
    return true;
}

bool lq_vl53l0x::write_static_init_sequence()
{
    for (unsigned i = 0; i < sizeof(LQ_VL53L0X_STATIC_INIT) / sizeof(LQ_VL53L0X_STATIC_INIT[0]); i++) {
        if (!write_reg(LQ_VL53L0X_STATIC_INIT[i].reg, LQ_VL53L0X_STATIC_INIT[i].value)) {
            printf("[VL53L0X] init reg 0x%02X failed\n", LQ_VL53L0X_STATIC_INIT[i].reg);
            return false;
        }
        usleep(30);
    }
    return true;
}

bool lq_vl53l0x::calibrate(uint8_t vhv_init_byte)
{
    if (!write_reg(VL53L0X_REG_SYSRANGE_START, static_cast<uint8_t>(0x01 | vhv_init_byte))) {
        printf("[VL53L0X] calibration start write failed (0x%02X)\n",
               vhv_init_byte);
        return false;
    }

    uint8_t int_status = 0;
    uint32_t read_errors = 0;
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(LQ_VL53L0X_IO_TIMEOUT_MS);
    while (std::chrono::steady_clock::now() < deadline) {
        if (!read_reg(VL53L0X_REG_RESULT_INTERRUPT_STATUS, int_status)) {
            ++read_errors;
            usleep(1000);
            continue;
        }
        if ((int_status & 0x07) != 0) break;
        usleep(1000);
    }
    if ((int_status & 0x07) == 0) {
        printf("[VL53L0X] calibration timeout "
               "(0x%02X status=0x%02X read_errors=%u)\n",
               vhv_init_byte, int_status, read_errors);
        clear_interrupt();
        write_reg(VL53L0X_REG_SYSRANGE_START, 0x00);
        return false;
    }

    return write_reg(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01) &&
           write_reg(VL53L0X_REG_SYSRANGE_START, 0x00);
}

bool lq_vl53l0x::init(bool power_2v8)
{
    if (!i2c_) {
        printf("[VL53L0X] No I2C object\n");
        return false;
    }

    uint8_t model = 0, rev = 0;
    get_model_info(model, rev);
    if (model != 0xEE && xshut_pin_) {
        hardware_reset();
        get_model_info(model, rev);
    }
    printf("[VL53L0X] model=0x%02X rev=0x%02X\n", model, rev);
    if (model != 0xEE) {
        printf("[VL53L0X] Device not found. Check wiring/power/address.\n");
        return false;
    }

    if (!set_flag(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, 0, power_2v8)) return false;

    if (!write_reg(0x88, 0x00) || !write_reg(0x80, 0x01) ||
        !write_reg(0xFF, 0x01) || !write_reg(0x00, 0x00)) return false;
    if (!read_reg(0x91, stop_variable_)) return false;
    if (!write_reg(0x00, 0x01) || !write_reg(0xFF, 0x00) ||
        !write_reg(0x80, 0x00)) return false;

    if (!set_flag(VL53L0X_REG_MSRC_CONFIG_CONTROL, 1, true) ||
        !set_flag(VL53L0X_REG_MSRC_CONFIG_CONTROL, 4, true)) return false;
    if (!set_signal_rate_limit(0.10f)) return false;
    if (!write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)) return false;

    uint8_t spad_count = 0;
    bool spad_is_aperture = false;
    if (!get_spad_info(spad_count, spad_is_aperture)) {
        printf("[VL53L0X] SPAD info timeout\n");
        return false;
    }

    uint8_t spad_map[6] = {0};
    if (!read_regs(VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6)) return false;

    if (!write_reg(0xFF, 0x01) || !write_reg(0x4F, 0x00) ||
        !write_reg(0x4E, 0x2C) || !write_reg(0xFF, 0x00) ||
        !write_reg(VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)) return false;

    uint8_t enabled = 0;
    const uint8_t first_spad = spad_is_aperture ? 12 : 0;
    for (uint8_t i = 0; i < 48; i++) {
        uint8_t mask = static_cast<uint8_t>(1u << (i & 0x07));
        if (i < first_spad || enabled >= spad_count) {
            spad_map[i >> 3] &= static_cast<uint8_t>(~mask);
        } else if (spad_map[i >> 3] & mask) {
            enabled++;
        }
    }
    if (!i2c_->write_bytes(VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6)) return false;

    if (!write_static_init_sequence()) return false;

    if (!write_reg(VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)) return false;
    if (!set_flag(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, 4, false)) return false;
    clear_interrupt();

    bool calibration_degraded = false;
    if (!write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01) ||
        !calibrate(0x40)) {
        printf("[VL53L0X] VHV calibration failed; retrying once\n");
        usleep(50000);
        if (!write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01) ||
            !calibrate(0x40)) {
            printf("[VL53L0X] VHV calibration unavailable; "
                   "continuing in degraded ranging mode\n");
            calibration_degraded = true;
        }
    }
    if (!write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02) ||
        !calibrate(0x00)) {
        printf("[VL53L0X] phase calibration failed; retrying once\n");
        usleep(50000);
        if (!write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02) ||
            !calibrate(0x00)) {
            printf("[VL53L0X] phase calibration unavailable; "
                   "continuing in degraded ranging mode\n");
            calibration_degraded = true;
        }
    }
    if (!write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8)) return false;

    initialized_ = true;
    printf("[VL53L0X] Init OK, spads=%u aperture=%u calibration=%s\n",
           spad_count, spad_is_aperture ? 1 : 0,
           calibration_degraded ? "degraded" : "full");
    return true;
}

bool lq_vl53l0x::set_signal_rate_limit(float limit_mcps)
{
    if (limit_mcps < 0.0f || limit_mcps > 511.99f) return false;
    uint16_t fixed_9_7 = static_cast<uint16_t>(limit_mcps * 128.0f);
    return write_reg16(VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN, fixed_9_7);
}

bool lq_vl53l0x::prepare_single_shot()
{
    return write_reg(0x80, 0x01) && write_reg(0xFF, 0x01) &&
           write_reg(0x00, 0x00) && write_reg(0x91, stop_variable_) &&
           write_reg(0x00, 0x01) && write_reg(0xFF, 0x00) &&
           write_reg(0x80, 0x00) &&
           write_reg(VL53L0X_REG_SYSRANGE_START, 0x01);
}

bool lq_vl53l0x::data_ready()
{
    uint8_t value = 0;
    return read_reg(VL53L0X_REG_RESULT_INTERRUPT_STATUS, value) && ((value & 0x07) != 0);
}

bool lq_vl53l0x::wait_data_ready(uint32_t timeout_ms)
{
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (data_ready()) return true;
        usleep(1000);
    }
    return false;
}

void lq_vl53l0x::clear_interrupt()
{
    write_reg(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

const char* lq_vl53l0x::status_text(uint8_t range_status, uint16_t distance_mm)
{
    if (distance_mm == 0 || distance_mm == 8190 || distance_mm == 8191 || distance_mm == 0xFFFF) {
        return "invalid distance";
    }
    if ((range_status & 0x78) != 0) return "range warning";
    return "range valid";
}

bool lq_vl53l0x::read_result(vl53l0x_result_t &result, bool blocking, uint32_t timeout_ms)
{
    std::memset(&result, 0, sizeof(result));
    result.status_text = "not initialized";
    if (!initialized_ || !i2c_) return false;

    if (!started_) {
        if (!prepare_single_shot()) {
            result.status_text = "start failed";
            return false;
        }

        uint8_t start = 0;
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            if (!read_reg(VL53L0X_REG_SYSRANGE_START, start)) {
                result.status_text = "i2c error";
                return false;
            }
            if ((start & 0x01) == 0) break;
            usleep(1000);
        }
        if ((start & 0x01) != 0) {
            result.status_text = "start timeout";
            return false;
        }
    }

    if (blocking && !wait_data_ready(timeout_ms)) {
        result.status_text = "data timeout";
        return false;
    }

    uint8_t int_status = 0;
    uint8_t range_status = 0;
    uint16_t distance = 0;
    if (!read_reg(VL53L0X_REG_RESULT_INTERRUPT_STATUS, int_status) ||
        !read_reg(VL53L0X_REG_RESULT_RANGE_STATUS, range_status) ||
        !read_reg16(static_cast<uint8_t>(VL53L0X_REG_RESULT_RANGE_STATUS + 10), distance)) {
        result.status_text = "i2c error";
        clear_interrupt();
        return false;
    }

    clear_interrupt();
    result.distance_mm = distance;
    result.range_status = range_status;
    result.interrupt_status = int_status;
    result.status_text = status_text(range_status, distance);
    result.valid = (int_status & 0x07) != 0 &&
                   distance != 0 && distance != 8190 &&
                   distance != 8191 && distance != 0xFFFF;
    return true;
}

uint16_t lq_vl53l0x::get_distance_mm(bool blocking, uint32_t timeout_ms)
{
    vl53l0x_result_t result;
    return read_result(result, blocking, timeout_ms) ? result.distance_mm : 0;
}

bool lq_vl53l0x::start_continuous(uint16_t period_ms)
{
    if (!initialized_ || !i2c_) return false;
    if (measuring_.load()) return true;
    if (!write_reg(0x80, 0x01) || !write_reg(0xFF, 0x01) ||
        !write_reg(0x00, 0x00) || !write_reg(0x91, stop_variable_) ||
        !write_reg(0x00, 0x01) || !write_reg(0xFF, 0x00) ||
        !write_reg(0x80, 0x00)) return false;

    if (period_ms > 0) {
        uint16_t osc = 0;
        uint32_t period_reg = period_ms;
        if (read_reg16(VL53L0X_REG_OSC_CALIBRATE_VAL, osc) && osc != 0) {
            period_reg *= osc;
        }
        if (period_reg > 0xFFFF) period_reg = 0xFFFF;
        if (!write_reg16(VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD,
                         static_cast<uint16_t>(period_reg))) return false;
        if (!write_reg(VL53L0X_REG_SYSRANGE_START, 0x04)) return false;
    } else {
        if (!write_reg(VL53L0X_REG_SYSRANGE_START, 0x02)) return false;
    }

    started_ = true;
    measure_period_ms_ = period_ms ? period_ms : 50;
    measuring_ = true;
    measure_thread_ = new std::thread([this]() {
        while (measuring_.load()) {
            vl53l0x_result_t result;
            if (read_result(result, true, 1000) && callback_) {
                callback_(result);
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(measure_period_ms_.load()));
        }
    });
    return true;
}

void lq_vl53l0x::stop_continuous()
{
    measuring_ = false;
    if (measure_thread_) {
        if (measure_thread_->joinable()) measure_thread_->join();
        delete measure_thread_;
        measure_thread_ = nullptr;
    }

    if (!i2c_) return;
    write_reg(VL53L0X_REG_SYSRANGE_START, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    write_reg(0x91, stop_variable_);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    started_ = false;
}
