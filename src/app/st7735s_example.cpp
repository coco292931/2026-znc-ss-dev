#include "st7735s.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

int parse_int(const char* text, int fallback) {
    if (!text) return fallback;
    char* end = nullptr;
    const long value = std::strtol(text, &end, 0);
    return end && *end == '\0' ? static_cast<int>(value) : fallback;
}

void usage(const char* executable) {
    std::printf(
        "Usage: %s [--spi /dev/spidev1.0] [--speed HZ] [--dc GPIO] [--rst GPIO]\n"
        "          [--rotation 0..3] [--x-offset N] [--y-offset N]\n",
        executable);
}

}  // namespace

int main(int argc, char** argv) {
    rewrite_path::St7735sConfig config;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            usage(argv[0]);
            return 0;
        }
        if (index + 1 >= argc) {
            usage(argv[0]);
            return 2;
        }
        const char* value = argv[++index];
        if (argument == "--spi") config.spi_device = value;
        else if (argument == "--speed") config.spi_speed_hz = parse_int(value, config.spi_speed_hz);
        else if (argument == "--dc") config.dc_gpio = parse_int(value, config.dc_gpio);
        else if (argument == "--rst") config.reset_gpio = parse_int(value, config.reset_gpio);
        else if (argument == "--rotation") config.rotation = parse_int(value, config.rotation);
        else if (argument == "--x-offset") config.x_offset = parse_int(value, config.x_offset);
        else if (argument == "--y-offset") config.y_offset = parse_int(value, config.y_offset);
        else {
            usage(argv[0]);
            return 2;
        }
    }

    rewrite_path::St7735s display(config);
    if (!display.initialize()) {
        std::fprintf(stderr, "ST7735S initialization failed: %s\n", display.last_error().c_str());
        return 1;
    }

    const std::uint16_t colors[] = {
        rewrite_path::St7735s::rgb565(255, 0, 0),
        rewrite_path::St7735s::rgb565(0, 255, 0),
        rewrite_path::St7735s::rgb565(0, 0, 255),
        rewrite_path::St7735s::rgb565(255, 255, 0),
        rewrite_path::St7735s::rgb565(0, 255, 255),
        rewrite_path::St7735s::rgb565(255, 0, 255),
        rewrite_path::St7735s::rgb565(255, 255, 255),
        rewrite_path::St7735s::rgb565(0, 0, 0),
    };
    const int bar_width = display.width() / 8;
    for (int index = 0; index < 8; ++index) {
        const int x = index * bar_width;
        const int width = index == 7 ? display.width() - x : bar_width;
        if (!display.fill_rect(x, 0, width, display.height(), colors[index])) {
            std::fprintf(stderr, "ST7735S draw failed: %s\n", display.last_error().c_str());
            return 1;
        }
    }

    const std::uint16_t black = rewrite_path::St7735s::rgb565(0, 0, 0);
    const std::uint16_t white = rewrite_path::St7735s::rgb565(255, 255, 255);
    if (!display.fill_rect(12, 20, display.width() - 24, 8, black) ||
        !display.fill_rect(12, display.height() - 28, display.width() - 24, 8, white)) {
        std::fprintf(stderr, "ST7735S draw failed: %s\n", display.last_error().c_str());
        return 1;
    }
    std::printf("ST7735S example displayed on %s (%dx%d, GPIO63 CS)\n",
                config.spi_device.c_str(),
                display.width(),
                display.height());
    return 0;
}


