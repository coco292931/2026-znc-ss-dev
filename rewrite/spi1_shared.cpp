#include "spi1_shared.hpp"

#include "LQ_HW_GPIO.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>

namespace rewrite_path {

namespace {

constexpr std::uintptr_t kPinmuxPageAddress = 0x16000000UL;
constexpr std::size_t kPinmuxPageSize = 0x10000UL;
constexpr std::size_t kSpi1PinmuxOffset = 0x49cUL;

void set_pinmux(volatile std::uint32_t* reg, int gpio, int mux) {
    const int shift = (gpio % 16) * 2;
    std::uint32_t value = *reg;
    value &= ~(0b11u << shift);
    value |= (static_cast<std::uint32_t>(mux) & 0b11u) << shift;
    *reg = value;
    (void)*reg;
}

}  // namespace

Spi1SharedBus::Spi1SharedBus() = default;

Spi1SharedBus::~Spi1SharedBus() {
    if (fd_ >= 0) ::close(fd_);
    if (pinmux_mapping_) ::munmap(pinmux_mapping_, kPinmuxPageSize);
}

void Spi1SharedBus::set_error(const std::string& message) {
    error_ = message;
}

bool Spi1SharedBus::open(const std::string& device,
                         int tft_cs_gpio,
                         int imu_cs_gpio) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ >= 0) {
        if (device_ == device && tft_cs_gpio_ == tft_cs_gpio &&
            imu_cs_gpio_ == imu_cs_gpio) {
            return true;
        }
        set_error("SPI1 is already open with different bus settings");
        return false;
    }

    const int fd = ::open(device.c_str(), O_RDWR);
    if (fd < 0) {
        set_error("Cannot open " + device + ": " + std::strerror(errno));
        return false;
    }

    std::uint8_t mode = SPI_MODE_0;
    std::uint8_t bits = 8;
    if (::ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        const int saved_errno = errno;
        ::close(fd);
        set_error("Cannot configure shared SPI1: " +
                  std::string(std::strerror(saved_errno)));
        return false;
    }

    const int mem_fd = ::open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        const int saved_errno = errno;
        ::close(fd);
        set_error("Cannot open SPI1 pinmux: " +
                  std::string(std::strerror(saved_errno)));
        return false;
    }
    void* pinmux_mapping = ::mmap(nullptr,
                                  kPinmuxPageSize,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED,
                                  mem_fd,
                                  static_cast<off_t>(kPinmuxPageAddress));
    ::close(mem_fd);
    if (pinmux_mapping == MAP_FAILED) {
        const int saved_errno = errno;
        ::close(fd);
        set_error("Cannot map SPI1 pinmux: " +
                  std::string(std::strerror(saved_errno)));
        return false;
    }
    auto* pinmux_register = reinterpret_cast<volatile std::uint32_t*>(
        static_cast<std::uint8_t*>(pinmux_mapping) + kSpi1PinmuxOffset);

    std::unique_ptr<HWGpio> tft_cs(
        new HWGpio(static_cast<std::uint8_t>(tft_cs_gpio), GPIO_Mode_Out));
    std::unique_ptr<HWGpio> imu_cs(
        new HWGpio(static_cast<std::uint8_t>(imu_cs_gpio), GPIO_Mode_Out));
    tft_cs->SetGpioValue(1);
    imu_cs->SetGpioValue(1);
    set_pinmux(pinmux_register, 60, 3);
    set_pinmux(pinmux_register, 61, 3);
    set_pinmux(pinmux_register, 62, 3);
    set_pinmux(pinmux_register, tft_cs_gpio, 0);
    set_pinmux(pinmux_register, imu_cs_gpio, 0);

    fd_ = fd;
    device_ = device;
    tft_cs_gpio_ = tft_cs_gpio;
    imu_cs_gpio_ = imu_cs_gpio;
    char link_target[256] = {};
    const ssize_t link_length =
        ::readlink(device.c_str(), link_target, sizeof(link_target) - 1);
    const std::string physical_device = link_length > 0
        ? std::string(link_target, static_cast<std::size_t>(link_length))
        : device;
    manual_tft_cs_ =
        physical_device.find("spidev1.2") != std::string::npos;
    tft_cs_ = std::move(tft_cs);
    imu_cs_ = std::move(imu_cs);
    pinmux_mapping_ = pinmux_mapping;
    pinmux_register_ = pinmux_register;
    error_.clear();
    return true;
}

void Spi1SharedBus::set_tft_cs_hardware(bool enabled) {
    set_pinmux(pinmux_register_, tft_cs_gpio_, enabled ? 3 : 0);
}

bool Spi1SharedBus::transfer(Spi1Chip chip,
                             const std::uint8_t* tx,
                             std::uint8_t* rx,
                             std::size_t length,
                             std::uint32_t speed_hz) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0 || !tft_cs_ || !imu_cs_) {
        set_error("Shared SPI1 is not open");
        return false;
    }
    if (length == 0) return true;
    if (length > UINT32_MAX) {
        set_error("SPI1 transfer is too large");
        return false;
    }
    if (!tx && !rx) {
        set_error("SPI1 transfer has no buffer");
        return false;
    }

    tft_cs_->SetGpioValue(1);
    imu_cs_->SetGpioValue(1);
    if (chip == Spi1Chip::Tft) {
        if (manual_tft_cs_) {
            tft_cs_->SetGpioValue(0);
        } else {
            set_tft_cs_hardware(true);
        }
    } else {
        set_tft_cs_hardware(false);
        imu_cs_->SetGpioValue(0);
    }

    spi_ioc_transfer transfer_desc{};
    transfer_desc.tx_buf = reinterpret_cast<std::uintptr_t>(tx);
    transfer_desc.rx_buf = reinterpret_cast<std::uintptr_t>(rx);
    transfer_desc.len = static_cast<std::uint32_t>(length);
    transfer_desc.speed_hz = speed_hz;
    transfer_desc.bits_per_word = 8;
    const int result = ::ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer_desc);
    const int saved_errno = errno;
    if (chip == Spi1Chip::Tft) {
        if (!manual_tft_cs_) set_tft_cs_hardware(false);
        tft_cs_->SetGpioValue(1);
    } else {
        imu_cs_->SetGpioValue(1);
    }
    if (result < 0) {
        set_error("SPI1 transfer failed: " +
                  std::string(std::strerror(saved_errno)));
        return false;
    }
    return true;
}

const std::string& Spi1SharedBus::last_error() const {
    return error_;
}

Spi1SharedBus& spi1_shared_bus() {
    static Spi1SharedBus bus;
    return bus;
}

}  // namespace rewrite_path
