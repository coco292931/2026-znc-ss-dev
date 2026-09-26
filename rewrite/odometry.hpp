#pragma once

#include "path_params.hpp"
#include "path_types.hpp"

namespace rewrite_path {

class PlanarOdometry {
public:
    explicit PlanarOdometry(const PathParams& params);

    void reset();
    OdometrySample update(const MotorFeedbackLite& motor,
                          const ImuFeedback& imu);
    const OdometrySample& latest() const { return sample_; }

private:
    const PathParams& p_;
    OdometrySample sample_;
    double previous_left_cm_ = 0.0;
    double previous_right_cm_ = 0.0;
    double previous_heading_deg_ = 0.0;
    bool initialized_ = false;
};

}  // namespace rewrite_path
