#pragma once

#include "path_params.hpp"
#include "path_types.hpp"

#include <memory>

namespace rewrite_path {

class ImuYawSensor {
public:
    explicit ImuYawSensor(const PathParams& params);
    ~ImuYawSensor();

    bool start();
    void stop();
    ImuFeedback read() const;

private:
    struct Impl;
    const PathParams& p_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rewrite_path
