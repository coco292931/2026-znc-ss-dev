#pragma once

#include <algorithm>
#include <cstdint>

namespace smartcar {

// LS2K0300用户手册21.3、21.4.2：FULL是R/W缓冲区，下降沿将内部周期
// 计数复制到FULL。消费后清零，下一笔相同周期也能被识别为新捕获。
// FULL为零仅表示暂无新完整周期，不能直接当作轮速为零。
class EncoderCapturePeriod {
public:
    void reset() {
        period_count_ = 0;
        captured_at_s_ = 0.0;
    }

    std::uint32_t read(volatile std::uint32_t& full_buffer,
                       double now_s,
                       double clock_hz) {
        const std::uint32_t captured = full_buffer;
        if (captured != 0) {
            // 仅消费非零值时清缓冲，不重置捕获控制器或内部计数器。
            full_buffer = 0;
            period_count_ = captured;
            captured_at_s_ = now_s;
        }
        if (period_count_ == 0 || clock_hz <= 0.0) return 0;

        // 容忍控制循环间隙，以及读取和清零之间丢失一笔捕获。
        // 低速时按两个实际脉冲周期等待，避免固定超时误判停转。
        const double period_s = static_cast<double>(period_count_) / clock_hz;
        const double timeout_s = std::max(0.05, 2.0 * period_s);
        return now_s - captured_at_s_ > timeout_s ? 0 : period_count_;
    }

private:
    std::uint32_t period_count_ = 0;
    double captured_at_s_ = 0.0;
};

}  // namespace smartcar
