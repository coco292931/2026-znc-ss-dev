#pragma once

#include <memory>
#include <string>

namespace smartcar {

struct WheelFeedback {
    double left_rpm = 0.0;
    double right_rpm = 0.0;
    bool valid = false;
};

struct MotorOutput {
    double left_percent = 0.0;
    double right_percent = 0.0;
};

struct HardwareConfig {
    // Verified wheel-side pairing used by lq_motor_test/lq_encoder_view:
    //   left:  PWM82/ATIM_CH2 + DIR21 + encoder PIN67/PWM_CH3
    //   right: PWM81/ATIM_CH1 + DIR22 + encoder PIN65/PWM_CH1
    // Never exchange only the PWM pins: that makes each wheel's duty command
    // use the opposite wheel while its direction pin remains local.
    int left_pwm_pin = 82;
    int left_pwm_channel = 2;
    int left_dir_pin = 21;
    bool left_motor_reversed = false;

    int right_pwm_pin = 81;
    int right_pwm_channel = 1;
    int right_dir_pin = 22;
    bool right_motor_reversed = true;

    int left_encoder_pin = 67;
    int left_encoder_channel = 3;
    int right_encoder_pin = 65;
    int right_encoder_channel = 1;

    double pwm_frequency_hz = 160000.0;
    int duty_max = 10000;
    double max_motor_percent = 55.0;

    double encoder_clock_hz = 160000000.0;
    int encoder_lines = 1024;
    double encoder_gear_ratio = 30.0 / 68.0;
    // The reference coefficient is converted to an elapsed-time response.
    double encoder_filter_alpha = 0.35;
    double encoder_filter_reference_hz = 50.0;
};

class VehicleHardware {
public:
    virtual ~VehicleHardware() = default;

    virtual bool start() = 0;
    virtual WheelFeedback read_feedback(double dt_seconds) = 0;
    virtual void set_motor_percent(double left_percent, double right_percent) = 0;
    virtual void stop() = 0;
    virtual bool motors_enabled() const = 0;
};

std::unique_ptr<VehicleHardware> make_vehicle_hardware(const HardwareConfig& config,
                                                       bool dry_run,
                                                       bool enable_motors);

std::string hardware_backend_name(bool dry_run, bool enable_motors);

}  // namespace smartcar
