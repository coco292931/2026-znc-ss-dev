#pragma once

#include <algorithm>
#include <cmath>

namespace smartcar {

inline double bound_encoder_rpm_magnitude(double sample_rpm,
                                          double filtered_rpm) {
    if (!std::isfinite(sample_rpm) || sample_rpm <= 0.0) {
        return 0.0;
    }

    const double filtered_magnitude = std::abs(filtered_rpm);
    const double reference_rpm = std::max(20.0, filtered_magnitude);
    const double max_delta_rpm = 2.0 * reference_rpm;
    return std::max(
        0.0,
        std::min(sample_rpm, filtered_magnitude + max_delta_rpm));
}

}  // namespace smartcar
