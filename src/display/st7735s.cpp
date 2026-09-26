#include "st7735s.hpp"

#include "spi1_shared.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "LQ_HW_GPIO.hpp"

namespace rewrite_path {

namespace {

constexpr std::uint8_t kSwReset = 0x01;
constexpr std::uint8_t kSleepOut = 0x11;
constexpr std::uint8_t kNormalOn = 0x13;
constexpr std::uint8_t kInvertOff = 0x20;
constexpr std::uint8_t kInvertOn = 0x21;
constexpr std::uint8_t kDisplayOn = 0x29;
constexpr std::uint8_t kColumnAddress = 0x2a;
constexpr std::uint8_t kRowAddress = 0x2b;
constexpr std::uint8_t kMemoryWrite = 0x2c;
constexpr std::uint8_t kMemoryAccess = 0x36;
constexpr std::uint8_t kPixelFormat = 0x3a;
constexpr std::size_t kSpidevChunkBytes = 4096;
constexpr const char* kDisplayLockPath = "/run/lock/smartcar-st7735s.lock";
constexpr const char* kDisplayLockFallback = "/tmp/smartcar-st7735s.lock";

void delay_ms(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

bool uses_vendor_tft18_profile(const std::string& device) {
    if (device.find("spidev1.2") != std::string::npos) return true;
    char target[256] = {};
    const ssize_t length =
        ::readlink(device.c_str(), target, sizeof(target) - 1);
    return length > 0 &&
        std::string(target, static_cast<std::size_t>(length))
            .find("spidev1.2") != std::string::npos;
}

// Compact 5x7 ASCII font packed as five column bytes per glyph, covering
// printable characters 0x20..0x7e. Each byte's low seven bits are the
// column pixels top-to-bottom. Rendered inside a 6x8 cell (one blank
// column and one blank row of spacing) so text tiles without gaps.
constexpr std::uint8_t kFont5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},  // space
    {0x00, 0x00, 0x5f, 0x00, 0x00},  // !
    {0x00, 0x07, 0x00, 0x07, 0x00},  // "
    {0x14, 0x7f, 0x14, 0x7f, 0x14},  // #
    {0x24, 0x2a, 0x7f, 0x2a, 0x12},  // $
    {0x23, 0x13, 0x08, 0x64, 0x62},  // %
    {0x36, 0x49, 0x55, 0x22, 0x50},  // &
    {0x00, 0x05, 0x03, 0x00, 0x00},  // '
    {0x00, 0x1c, 0x22, 0x41, 0x00},  // (
    {0x00, 0x41, 0x22, 0x1c, 0x00},  // )
    {0x14, 0x08, 0x3e, 0x08, 0x14},  // *
    {0x08, 0x08, 0x3e, 0x08, 0x08},  // +
    {0x00, 0x50, 0x30, 0x00, 0x00},  // ,
    {0x08, 0x08, 0x08, 0x08, 0x08},  // -
    {0x00, 0x60, 0x60, 0x00, 0x00},  // .
    {0x20, 0x10, 0x08, 0x04, 0x02},  // /
    {0x3e, 0x51, 0x49, 0x45, 0x3e},  // 0
    {0x00, 0x42, 0x7f, 0x40, 0x00},  // 1
    {0x42, 0x61, 0x51, 0x49, 0x46},  // 2
    {0x21, 0x41, 0x45, 0x4b, 0x31},  // 3
    {0x18, 0x14, 0x12, 0x7f, 0x10},  // 4
    {0x27, 0x45, 0x45, 0x45, 0x39},  // 5
    {0x3c, 0x4a, 0x49, 0x49, 0x30},  // 6
    {0x01, 0x71, 0x09, 0x05, 0x03},  // 7
    {0x36, 0x49, 0x49, 0x49, 0x36},  // 8
    {0x06, 0x49, 0x49, 0x29, 0x1e},  // 9
    {0x00, 0x36, 0x36, 0x00, 0x00},  // :
    {0x00, 0x56, 0x36, 0x00, 0x00},  // ;
    {0x08, 0x14, 0x22, 0x41, 0x00},  // <
    {0x14, 0x14, 0x14, 0x14, 0x14},  // =
    {0x00, 0x41, 0x22, 0x14, 0x08},  // >
    {0x02, 0x01, 0x51, 0x09, 0x06},  // ?
    {0x32, 0x49, 0x79, 0x41, 0x3e},  // @
    {0x7e, 0x11, 0x11, 0x11, 0x7e},  // A
    {0x7f, 0x49, 0x49, 0x49, 0x36},  // B
    {0x3e, 0x41, 0x41, 0x41, 0x22},  // C
    {0x7f, 0x41, 0x41, 0x22, 0x1c},  // D
    {0x7f, 0x49, 0x49, 0x49, 0x41},  // E
    {0x7f, 0x09, 0x09, 0x09, 0x01},  // F
    {0x3e, 0x41, 0x49, 0x49, 0x7a},  // G
    {0x7f, 0x08, 0x08, 0x08, 0x7f},  // H
    {0x00, 0x41, 0x7f, 0x41, 0x00},  // I
    {0x20, 0x40, 0x41, 0x3f, 0x01},  // J
    {0x7f, 0x08, 0x14, 0x22, 0x41},  // K
    {0x7f, 0x40, 0x40, 0x40, 0x40},  // L
    {0x7f, 0x02, 0x0c, 0x02, 0x7f},  // M
    {0x7f, 0x04, 0x08, 0x10, 0x7f},  // N
    {0x3e, 0x41, 0x41, 0x41, 0x3e},  // O
    {0x7f, 0x09, 0x09, 0x09, 0x06},  // P
    {0x3e, 0x41, 0x51, 0x21, 0x5e},  // Q
    {0x7f, 0x09, 0x19, 0x29, 0x46},  // R
    {0x46, 0x49, 0x49, 0x49, 0x31},  // S
    {0x01, 0x01, 0x7f, 0x01, 0x01},  // T
    {0x3f, 0x40, 0x40, 0x40, 0x3f},  // U
    {0x1f, 0x20, 0x40, 0x20, 0x1f},  // V
    {0x3f, 0x40, 0x38, 0x40, 0x3f},  // W
    {0x63, 0x14, 0x08, 0x14, 0x63},  // X
    {0x07, 0x08, 0x70, 0x08, 0x07},  // Y
    {0x61, 0x51, 0x49, 0x45, 0x43},  // Z
    {0x00, 0x7f, 0x41, 0x41, 0x00},  // [
    {0x02, 0x04, 0x08, 0x10, 0x20},  // backslash
    {0x00, 0x41, 0x41, 0x7f, 0x00},  // ]
    {0x04, 0x02, 0x01, 0x02, 0x04},  // ^
    {0x40, 0x40, 0x40, 0x40, 0x40},  // _
    {0x00, 0x01, 0x02, 0x04, 0x00},  // `
    {0x20, 0x54, 0x54, 0x54, 0x78},  // a
    {0x7f, 0x48, 0x44, 0x44, 0x38},  // b
    {0x38, 0x44, 0x44, 0x44, 0x20},  // c
    {0x38, 0x44, 0x44, 0x48, 0x7f},  // d
    {0x38, 0x54, 0x54, 0x54, 0x18},  // e
    {0x08, 0x7e, 0x09, 0x01, 0x02},  // f
    {0x0c, 0x52, 0x52, 0x52, 0x3e},  // g
    {0x7f, 0x08, 0x04, 0x04, 0x78},  // h
    {0x00, 0x44, 0x7d, 0x40, 0x00},  // i
    {0x20, 0x40, 0x44, 0x3d, 0x00},  // j
    {0x7f, 0x10, 0x28, 0x44, 0x00},  // k
    {0x00, 0x41, 0x7f, 0x40, 0x00},  // l
    {0x7c, 0x04, 0x18, 0x04, 0x78},  // m
    {0x7c, 0x08, 0x04, 0x04, 0x78},  // n
    {0x38, 0x44, 0x44, 0x44, 0x38},  // o
    {0x7c, 0x14, 0x14, 0x14, 0x08},  // p
    {0x08, 0x14, 0x14, 0x18, 0x7c},  // q
    {0x7c, 0x08, 0x04, 0x04, 0x08},  // r
    {0x48, 0x54, 0x54, 0x54, 0x20},  // s
    {0x04, 0x3f, 0x44, 0x40, 0x20},  // t
    {0x3c, 0x40, 0x40, 0x20, 0x7c},  // u
    {0x1c, 0x20, 0x40, 0x20, 0x1c},  // v
    {0x3c, 0x40, 0x30, 0x40, 0x3c},  // w
    {0x44, 0x28, 0x10, 0x28, 0x44},  // x
    {0x0c, 0x50, 0x50, 0x50, 0x3c},  // y
    {0x44, 0x64, 0x54, 0x4c, 0x44},  // z
    {0x00, 0x08, 0x36, 0x41, 0x00},  // {
    {0x00, 0x00, 0x7f, 0x00, 0x00},  // |
    {0x00, 0x41, 0x36, 0x08, 0x00},  // }
    {0x08, 0x04, 0x08, 0x10, 0x08},  // ~
};

}  // namespace

struct St7735s::Impl {
    explicit Impl(St7735sConfig new_config)
        : config(std::move(new_config)),
          vendor_tft18_profile(uses_vendor_tft18_profile(config.spi_device)) {}

    ~Impl() {
        if (lock_fd >= 0) {
            (void)::flock(lock_fd, LOCK_UN);
            ::close(lock_fd);
        }
    }

    void set_error(const std::string& message) {
        error = message;
    }

    bool transfer(const std::uint8_t* data, std::size_t length) {
        if (length == 0) return true;
        if (!spi1_shared_bus().transfer(
                Spi1Chip::Tft, data, nullptr, length, config.spi_speed_hz)) {
            set_error(spi1_shared_bus().last_error());
            return false;
        }
        return true;
    }

    bool command(std::uint8_t value) {
        if (!dc) {
            set_error("Display GPIOs are not open");
            return false;
        }
        dc->SetGpioValue(0);
        return transfer(&value, 1);
    }

    bool data(const std::uint8_t* bytes, std::size_t length) {
        if (!dc) {
            set_error("Display GPIOs are not open");
            return false;
        }
        dc->SetGpioValue(1);
        return transfer(bytes, length);
    }

    bool command_data(std::uint8_t command_value,
                      const std::uint8_t* bytes,
                      std::size_t length) {
        return command(command_value) && data(bytes, length);
    }

    bool set_window(int x0, int y0, int x1, int y1) {
        x0 += config.x_offset;
        x1 += config.x_offset;
        y0 += config.y_offset;
        y1 += config.y_offset;

        const std::uint8_t columns[] = {
            static_cast<std::uint8_t>(x0 >> 8),
            static_cast<std::uint8_t>(x0),
            static_cast<std::uint8_t>(x1 >> 8),
            static_cast<std::uint8_t>(x1),
        };
        const std::uint8_t rows[] = {
            static_cast<std::uint8_t>(y0 >> 8),
            static_cast<std::uint8_t>(y0),
            static_cast<std::uint8_t>(y1 >> 8),
            static_cast<std::uint8_t>(y1),
        };
        return command_data(kColumnAddress, columns, sizeof(columns)) &&
               command_data(kRowAddress, rows, sizeof(rows)) &&
               command(kMemoryWrite);
    }

    St7735sConfig config;
    bool device_open = false;
    int logical_width = 128;
    int logical_height = 160;
    bool initialized = false;
    bool vendor_tft18_profile = false;
    std::string error;
    int lock_fd = -1;
    std::unique_ptr<HWGpio> dc;
    std::unique_ptr<HWGpio> reset;
};

St7735s::St7735s(St7735sConfig config)
    : impl_(new Impl(std::move(config))) {
    impl_->logical_width = impl_->config.width;
    impl_->logical_height = impl_->config.height;
}

St7735s::~St7735s() = default;

bool St7735s::open_device() {
    if (impl_->device_open) return true;
    int lock_fd = ::open(kDisplayLockPath, O_RDWR | O_CREAT | O_CLOEXEC, 0666);
    if (lock_fd < 0 && errno == ENOENT) {
        lock_fd = ::open(
            kDisplayLockFallback, O_RDWR | O_CREAT | O_CLOEXEC, 0666);
    }
    if (lock_fd < 0) {
        impl_->set_error("Cannot open TFT ownership lock: " +
                         std::string(std::strerror(errno)));
        return false;
    }
    if (::flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
        const int saved_errno = errno;
        ::close(lock_fd);
        impl_->set_error(
            saved_errno == EWOULDBLOCK
                ? "TFT is already owned by another process"
                : "Cannot lock TFT ownership: " +
                      std::string(std::strerror(saved_errno)));
        return false;
    }
    if (!spi1_shared_bus().open(impl_->config.spi_device,
                                impl_->config.cs_gpio,
                                impl_->config.imu_cs_gpio)) {
        (void)::flock(lock_fd, LOCK_UN);
        ::close(lock_fd);
        impl_->set_error(spi1_shared_bus().last_error());
        return false;
    }

    impl_->dc.reset(new HWGpio(static_cast<std::uint8_t>(impl_->config.dc_gpio),
                               GPIO_Mode_Out));
    impl_->reset.reset(new HWGpio(static_cast<std::uint8_t>(impl_->config.reset_gpio),
                                  GPIO_Mode_Out));
    impl_->dc->SetGpioValue(1);
    impl_->reset->SetGpioValue(1);
    impl_->lock_fd = lock_fd;
    impl_->device_open = true;
    return true;
}

bool St7735s::initialize() {
    if (impl_->initialized) return true;
    if (!open_device()) return false;

    impl_->reset->SetGpioValue(1);
    delay_ms(10);
    impl_->reset->SetGpioValue(0);
    delay_ms(120);
    impl_->reset->SetGpioValue(1);
    delay_ms(120);

    if (impl_->vendor_tft18_profile) {
        const std::uint8_t pixel_format[] = {0x55};
        const std::uint8_t gamma_curve[] = {0x04};
        const std::uint8_t gamma_adjust[] = {0x01};
        const std::uint8_t positive_gamma[] = {
            0x3f, 0x25, 0x1c, 0x1e, 0x20, 0x12, 0x2a, 0x90,
            0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
        };
        const std::uint8_t negative_gamma[] = {
            0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xa7,
            0x3d, 0x18, 0x25, 0x2a, 0x2b, 0x2b, 0x3a,
        };
        const std::uint8_t frame_rate[] = {0x00, 0x00};
        const std::uint8_t inversion_control[] = {0x07};
        const std::uint8_t power_control_1[] = {0x0a, 0x02};
        const std::uint8_t power_control_2[] = {0x02};
        const std::uint8_t vcom_control[] = {0x4f, 0x5a};
        const std::uint8_t vcom_offset[] = {0x40};
        const std::uint8_t entry_mode[] = {0x00};

        if (!impl_->command(kSleepOut)) return false;
        delay_ms(50);
        if (!impl_->command_data(kPixelFormat, pixel_format,
                                 sizeof(pixel_format)) ||
            !impl_->command_data(0x26, gamma_curve, sizeof(gamma_curve)) ||
            !impl_->command_data(0xf2, gamma_adjust,
                                 sizeof(gamma_adjust)) ||
            !impl_->command_data(0xe0, positive_gamma,
                                 sizeof(positive_gamma)) ||
            !impl_->command_data(0xe1, negative_gamma,
                                 sizeof(negative_gamma)) ||
            !impl_->command_data(0xb1, frame_rate, sizeof(frame_rate)) ||
            !impl_->command_data(0xb4, inversion_control,
                                 sizeof(inversion_control)) ||
            !impl_->command_data(0xc0, power_control_1,
                                 sizeof(power_control_1)) ||
            !impl_->command_data(0xc1, power_control_2,
                                 sizeof(power_control_2)) ||
            !impl_->command_data(0xc5, vcom_control,
                                 sizeof(vcom_control)) ||
            !impl_->command_data(0xc7, vcom_offset,
                                 sizeof(vcom_offset)) ||
            !set_rotation(impl_->config.rotation) ||
            !impl_->command_data(0xb7, entry_mode, sizeof(entry_mode)) ||
            !impl_->command(kDisplayOn)) {
            return false;
        }
        delay_ms(100);
        impl_->initialized = true;
        return true;
    }

    if (!impl_->command(kSwReset)) return false;
    delay_ms(150);
    if (!impl_->command(kSleepOut)) return false;
    delay_ms(150);

    const std::uint8_t frame_rate[] = {0x01, 0x2c, 0x2d};
    const std::uint8_t frame_rate_idle[] = {0x01, 0x2c, 0x2d};
    const std::uint8_t frame_rate_partial[] = {0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d};
    const std::uint8_t inversion_control[] = {0x07};
    const std::uint8_t power_control_1[] = {0xa2, 0x02, 0x84};
    const std::uint8_t power_control_2[] = {0xc5};
    const std::uint8_t power_control_3[] = {0x0a, 0x00};
    const std::uint8_t power_control_4[] = {0x8a, 0x2a};
    const std::uint8_t power_control_5[] = {0x8a, 0xee};
    const std::uint8_t vcom_control[] = {0x0e};
    const std::uint8_t pixel_format[] = {0x05};

    if (!impl_->command_data(0xb1, frame_rate, sizeof(frame_rate)) ||
        !impl_->command_data(0xb2, frame_rate_idle, sizeof(frame_rate_idle)) ||
        !impl_->command_data(0xb3, frame_rate_partial, sizeof(frame_rate_partial)) ||
        !impl_->command_data(0xb4, inversion_control, sizeof(inversion_control)) ||
        !impl_->command_data(0xc0, power_control_1, sizeof(power_control_1)) ||
        !impl_->command_data(0xc1, power_control_2, sizeof(power_control_2)) ||
        !impl_->command_data(0xc2, power_control_3, sizeof(power_control_3)) ||
        !impl_->command_data(0xc3, power_control_4, sizeof(power_control_4)) ||
        !impl_->command_data(0xc4, power_control_5, sizeof(power_control_5)) ||
        !impl_->command_data(0xc5, vcom_control, sizeof(vcom_control)) ||
        !impl_->command_data(kPixelFormat, pixel_format, sizeof(pixel_format)) ||
        !set_rotation(impl_->config.rotation) ||
        !set_inverted(false) ||
        !impl_->command(kNormalOn)) {
        return false;
    }
    delay_ms(10);
    if (!impl_->command(kDisplayOn)) return false;
    delay_ms(100);
    impl_->initialized = true;
    return true;
}

bool St7735s::set_rotation(int rotation) {
    rotation = ((rotation % 4) + 4) % 4;
    std::uint8_t memory_access = 0x00;
    switch (rotation) {
        case 0:
            memory_access = 0xc0;
            impl_->logical_width = impl_->config.width;
            impl_->logical_height = impl_->config.height;
            break;
        case 1:
            memory_access = 0xa0;
            impl_->logical_width = impl_->config.height;
            impl_->logical_height = impl_->config.width;
            break;
        case 2:
            memory_access = 0x00;
            impl_->logical_width = impl_->config.width;
            impl_->logical_height = impl_->config.height;
            break;
        case 3:
            memory_access = 0x60;
            impl_->logical_width = impl_->config.height;
            impl_->logical_height = impl_->config.width;
            break;
    }
    if (impl_->config.bgr) memory_access |= 0x08u;
    impl_->config.rotation = rotation;
    return impl_->command_data(kMemoryAccess, &memory_access, 1);
}

bool St7735s::set_inverted(bool inverted) {
    return impl_->command(inverted ? kInvertOn : kInvertOff);
}

bool St7735s::fill(std::uint16_t color) {
    return fill_rect(0, 0, width(), height(), color);
}

bool St7735s::fill_rect(int x,
                       int y,
                       int rect_width,
                       int rect_height,
                       std::uint16_t color) {
    if (!impl_->initialized) {
        impl_->set_error("Display is not initialized");
        return false;
    }
    if (rect_width <= 0 || rect_height <= 0 || x >= width() || y >= height() ||
        x + rect_width <= 0 || y + rect_height <= 0) {
        return true;
    }

    const int x0 = std::max(0, x);
    const int y0 = std::max(0, y);
    const int x1 = std::min(width() - 1, x + rect_width - 1);
    const int y1 = std::min(height() - 1, y + rect_height - 1);
    const int clipped_width = x1 - x0 + 1;
    const std::size_t row_bytes = static_cast<std::size_t>(clipped_width) * 2;
    const int rows_per_chunk =
        std::max(1, static_cast<int>(kSpidevChunkBytes / row_bytes));
    for (int chunk_y = y0; chunk_y <= y1; chunk_y += rows_per_chunk) {
        const int chunk_rows = std::min(rows_per_chunk, y1 - chunk_y + 1);
        if (!impl_->set_window(x0, chunk_y, x1, chunk_y + chunk_rows - 1)) return false;

        const std::size_t chunk_pixels =
            static_cast<std::size_t>(clipped_width) * static_cast<std::size_t>(chunk_rows);
        std::vector<std::uint8_t> bytes(chunk_pixels * 2);
        for (std::size_t index = 0; index < chunk_pixels; ++index) {
            bytes[index * 2] = static_cast<std::uint8_t>(color >> 8);
            bytes[index * 2 + 1] = static_cast<std::uint8_t>(color);
        }
        if (!impl_->data(bytes.data(), bytes.size())) return false;
    }
    return true;
}

bool St7735s::write_rgb565(int x,
                          int y,
                          int image_width,
                          int image_height,
                          const std::uint16_t* pixels,
                          std::size_t pixel_count) {
    if (!impl_->initialized) {
        impl_->set_error("Display is not initialized");
        return false;
    }
    if (!pixels || image_width <= 0 || image_height <= 0 || x < 0 || y < 0 ||
        x + image_width > width() || y + image_height > height()) {
        impl_->set_error("Invalid RGB565 image region");
        return false;
    }
    const std::size_t required =
        static_cast<std::size_t>(image_width) * static_cast<std::size_t>(image_height);
    if (pixel_count < required) {
        impl_->set_error("RGB565 pixel buffer is too small");
        return false;
    }
    const std::size_t row_bytes = static_cast<std::size_t>(image_width) * 2;
    const int rows_per_chunk =
        std::max(1, static_cast<int>(kSpidevChunkBytes / row_bytes));
    for (int row = 0; row < image_height; row += rows_per_chunk) {
        const int chunk_rows = std::min(rows_per_chunk, image_height - row);
        if (!impl_->set_window(x,
                               y + row,
                               x + image_width - 1,
                               y + row + chunk_rows - 1)) {
            return false;
        }

        const std::size_t chunk_pixels =
            static_cast<std::size_t>(image_width) * static_cast<std::size_t>(chunk_rows);
        std::vector<std::uint8_t> bytes(chunk_pixels * 2);
        const std::size_t source_offset = static_cast<std::size_t>(row) * image_width;
        for (std::size_t index = 0; index < chunk_pixels; ++index) {
            const std::uint16_t pixel = pixels[source_offset + index];
            bytes[index * 2] = static_cast<std::uint8_t>(pixel >> 8);
            bytes[index * 2 + 1] = static_cast<std::uint8_t>(pixel);
        }
        if (!impl_->data(bytes.data(), bytes.size())) return false;
    }
    return true;
}

bool St7735s::draw_char(int x, int y, char glyph, std::uint16_t fg,
                        std::uint16_t bg, int scale) {
    if (!impl_->initialized) {
        impl_->set_error("Display is not initialized");
        return false;
    }
    if (scale < 1) scale = 1;
    const int cell_width = kGlyphWidth * scale;
    const int cell_height = kGlyphHeight * scale;
    if (x < 0 || y < 0 || x + cell_width > width() || y + cell_height > height()) {
        return true;  // Silently clip glyphs that fall off the panel.
    }

    const auto code = static_cast<unsigned char>(glyph);
    const std::uint8_t* columns =
        (code >= 0x20 && code <= 0x7e) ? kFont5x7[code - 0x20] : kFont5x7[0];

    // Build the whole cell in one RGB565 buffer, then blit it with a single
    // windowed transfer so per-glyph SPI cost stays predictable.
    const std::size_t pixels =
        static_cast<std::size_t>(cell_width) * static_cast<std::size_t>(cell_height);
    std::vector<std::uint16_t> cell(pixels, bg);
    for (int col = 0; col < 5; ++col) {
        const std::uint8_t bits = columns[col];
        for (int row = 0; row < 7; ++row) {
            if (!(bits & (1u << row))) continue;
            for (int sy = 0; sy < scale; ++sy) {
                const int py = row * scale + sy;
                std::uint16_t* dst = &cell[static_cast<std::size_t>(py) * cell_width +
                                           static_cast<std::size_t>(col * scale)];
                for (int sx = 0; sx < scale; ++sx) dst[sx] = fg;
            }
        }
    }
    return write_rgb565(x, y, cell_width, cell_height, cell.data(), cell.size());
}

bool St7735s::draw_text(int x, int y, const std::string& text, std::uint16_t fg,
                        std::uint16_t bg, int scale) {
    if (scale < 1) scale = 1;
    const int advance = kGlyphWidth * scale;
    int cursor_x = x;
    for (char glyph : text) {
        if (!draw_char(cursor_x, y, glyph, fg, bg, scale)) return false;
        cursor_x += advance;
    }
    return true;
}

bool St7735s::ready() const {
    return impl_->initialized;
}

int St7735s::width() const {
    return impl_->logical_width;
}

int St7735s::height() const {
    return impl_->logical_height;
}

const std::string& St7735s::last_error() const {
    return impl_->error;
}

}  // namespace rewrite_path
