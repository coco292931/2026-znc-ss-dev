#pragma once

#include "path_params.hpp"
#include "path_types.hpp"

#include <memory>
#include <vector>

namespace rewrite_path {

class TofSlopeDetector {
public:
    explicit TofSlopeDetector(const PathParams& params);

    void reset();
    TofSlopeFeedback update(const TofDistanceSample& sample, double dt);

private:
    void set_ramp_active(bool active);

    const PathParams& p_;
    std::vector<double> baseline_samples_;
    std::uint64_t last_sequence_ = 0;
    double filtered_distance_mm_ = 0.0;
    double baseline_distance_mm_ = 0.0;
    double ramp_active_s_ = 0.0;
    int enter_count_ = 0;
    int exit_count_ = 0;
    bool filter_ready_ = false;
    bool baseline_ready_ = false;
    bool ramp_active_ = false;
    bool rearm_blocked_ = false;
};

bool apply_tof_stop_latch(const PathParams& params,
                          const TofSlopeFeedback& tof,
                          bool* stop_latched,
                          NavigationCommand* navigation);

class Vl53l0xDistanceSensor {
public:
    explicit Vl53l0xDistanceSensor(const PathParams& params);
    ~Vl53l0xDistanceSensor();

    bool start();
    void stop();
    TofDistanceSample read() const;

private:
    struct Impl;
    const PathParams& p_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rewrite_path
