#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

class HWGpio;

namespace rewrite_path {

enum class Spi1Chip {
    Tft,
    Imu,
};

class Spi1SharedBus {
public:
    Spi1SharedBus();
    ~Spi1SharedBus();

    Spi1SharedBus(const Spi1SharedBus&) = delete;
    Spi1SharedBus& operator=(const Spi1SharedBus&) = delete;

    bool open(const std::string& device,
              int tft_cs_gpio = 63,
              int imu_cs_gpio = 25);
    bool transfer(Spi1Chip chip,
                  const std::uint8_t* tx,
                  std::uint8_t* rx,
                  std::size_t length,
                  std::uint32_t speed_hz);
    const std::string& last_error() const;

private:
    void set_error(const std::string& message);
    void set_tft_cs_hardware(bool enabled);

    mutable std::mutex mutex_;
    int fd_ = -1;
    int tft_cs_gpio_ = -1;
    int imu_cs_gpio_ = -1;
    bool manual_tft_cs_ = false;
    std::string device_;
    std::string error_;
    std::unique_ptr<HWGpio> tft_cs_;
    std::unique_ptr<HWGpio> imu_cs_;
    void* pinmux_mapping_ = nullptr;
    volatile std::uint32_t* pinmux_register_ = nullptr;
};

Spi1SharedBus& spi1_shared_bus();

}  // namespace rewrite_path
