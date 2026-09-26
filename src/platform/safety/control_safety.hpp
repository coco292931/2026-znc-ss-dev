#pragma once

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>

namespace smartcar {

inline bool finite_motor_pair(double left, double right) {
    return std::isfinite(left) && std::isfinite(right);
}

// Reject partial parses, overflow, underflow and embedded NUL bytes.
inline bool parse_finite_double(const std::string& text, double* out) {
    if (!out || text.empty() || text.find('\0') != std::string::npos) return false;
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() ||
        end != text.c_str() + text.size() || !std::isfinite(value)) return false;
    *out = value;
    return true;
}

inline bool parse_integer(const std::string& text, int* out) {
    if (!out || text.empty() || text.find('\0') != std::string::npos) return false;
    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || end != text.c_str() + text.size() ||
        value < INT_MIN || value > INT_MAX) return false;
    *out = static_cast<int>(value);
    return true;
}

// Exactly one L=<number> and one R=<number>, separated by whitespace.
// Parse into temporaries: a bad packet never partially changes the command.
inline bool parse_motor_payload(const std::string& text, double* left, double* right) {
    if (!left || !right || text.size() > 255 ||
        text.find('\0') != std::string::npos) return false;
    std::istringstream input(text);
    std::string field;
    bool seen_left = false, seen_right = false;
    double l = 0.0, r = 0.0;
    while (input >> field) {
        if (field.size() < 3 || field[1] != '=') return false;
        double value = 0.0;
        if (!parse_finite_double(field.substr(2), &value)) return false;
        if (field[0] == 'L' && !seen_left) { l = value; seen_left = true; }
        else if (field[0] == 'R' && !seen_right) { r = value; seen_right = true; }
        else return false;
    }
    if (!seen_left || !seen_right) return false;
    *left = l;
    *right = r;
    return true;
}

// Legacy alpha is defined at 50 Hz; convert it to a physical time constant.
inline double encoder_alpha_for_dt(double reference_alpha, double dt, double reference_hz = 50.0) {
    if (!std::isfinite(reference_alpha) || !std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(reference_hz) || reference_hz <= 0.0) return 0.0;
    if (reference_alpha <= 0.0) return 0.0;
    if (reference_alpha >= 1.0) return 1.0;
    return -std::expm1(std::log1p(-reference_alpha) * dt * reference_hz);
}

}  // namespace smartcar
