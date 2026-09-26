#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace smartcar {

struct SuccessMotorPins {
    const char* name = "";
    int pwm_pin = 0;
    int pwm_channel = 0;
    int dir_pin = 0;
    bool reversed = false;
};

struct SuccessMotorConfig {
    double pwm_frequency_hz = 160000.0;
    int duty_max = 10000;
    double max_speed_percent = 55.0;
};

// Compatibility wrapper for the successful lq_motor examples.
//
// Public speed is the same style as success/lq_motor_test_main.cpp:
// set_speed(-55..55). Internally the ATIM channel is kept in PWM mode 2 with
// normal polarity, so speed 0 maps to the stop duty, not to a free-running pin.
class SuccessMotor {
public:
    SuccessMotor(SuccessMotorPins pins, SuccessMotorConfig config);
    ~SuccessMotor();

    SuccessMotor(const SuccessMotor&) = delete;
    SuccessMotor& operator=(const SuccessMotor&) = delete;

    bool start();
    void set_speed(double speed_percent);
    void stop();

    double last_speed_percent() const;
    const SuccessMotorPins& pins() const;

private:
    std::uint32_t duty_for_speed(double bounded_speed) const;

    SuccessMotorPins pins_;
    SuccessMotorConfig config_;
    double last_speed_percent_ = 0.0;
    bool faulted_ = false;

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace smartcar
