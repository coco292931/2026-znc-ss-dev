#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace rewrite_path {

struct St7735sConfig {
    std::string spi_device = "/dev/spidev1.0";
    std::uint32_t spi_speed_hz = 8000000;
    int cs_gpio = 63;
    int imu_cs_gpio = 25;
    int dc_gpio = 48;
    int reset_gpio = 49;
    int width = 128;
    int height = 160;
    int x_offset = 0;
    int y_offset = 0;
    int rotation = 0;
    // MADCTL color order. The on-board panel is RGB; set true only for a
    // module whose controller expects BGR ordering.
    bool bgr = false;
};

class St7735s {
public:
    explicit St7735s(St7735sConfig config = {});
    ~St7735s();

    St7735s(const St7735s&) = delete;
    St7735s& operator=(const St7735s&) = delete;

    bool open_device();
    bool initialize();
    bool set_rotation(int rotation);
    bool set_inverted(bool inverted);
    bool fill(std::uint16_t color);
    bool fill_rect(int x, int y, int width, int height, std::uint16_t color);
    bool write_rgb565(int x,
                      int y,
                      int width,
                      int height,
                      const std::uint16_t* pixels,
                      std::size_t pixel_count);

    // Renders one printable ASCII glyph in a 6x8 cell scaled by `scale`.
    // Foreground pixels use `fg`; the cell background uses `bg`. A single
    // SPI transfer per glyph keeps display refresh off the control loop.
    bool draw_char(int x, int y, char glyph, std::uint16_t fg,
                   std::uint16_t bg, int scale = 1);
    // Renders a left-to-right ASCII string; returns false on the first
    // glyph that fails. Newlines and clipping are the caller's concern.
    bool draw_text(int x, int y, const std::string& text, std::uint16_t fg,
                   std::uint16_t bg, int scale = 1);

    static constexpr int kGlyphWidth = 6;
    static constexpr int kGlyphHeight = 8;

    bool ready() const;
    int width() const;
    int height() const;
    const std::string& last_error() const;

    static constexpr std::uint16_t rgb565(std::uint8_t red,
                                           std::uint8_t green,
                                           std::uint8_t blue) {
        return static_cast<std::uint16_t>(((red & 0xf8u) << 8) |
                                          ((green & 0xfcu) << 3) |
                                          (blue >> 3));
    }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rewrite_path
