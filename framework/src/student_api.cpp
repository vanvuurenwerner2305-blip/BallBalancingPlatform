#include <cmath>

#include "bbp/bbp.hpp"
#include "runtime_hooks.hpp"

namespace bbp {

namespace {
inline int r5(const std::uint8_t* p) { return p[0] >> 3; }
inline int g6(const std::uint8_t* p) { return ((p[0] & 7) << 3) | (p[1] >> 5); }
inline int b5(const std::uint8_t* p) { return p[1] & 31; }
}  // namespace

// format: 0 = Gray8, 1 = RGB565 (big-endian), 2 = RGB888 (matches sim::PixelFormat)
int Image::red(int x, int y) const {
    const std::size_t i = std::size_t(y) * width_ + x;
    if (format_ == 0) return data_[i];
    if (format_ == 1) return (r5(data_ + 2 * i) * 255 + 15) / 31;
    return data_[3 * i];
}

int Image::green(int x, int y) const {
    const std::size_t i = std::size_t(y) * width_ + x;
    if (format_ == 0) return data_[i];
    if (format_ == 1) return (g6(data_ + 2 * i) * 255 + 31) / 63;
    return data_[3 * i + 1];
}

int Image::blue(int x, int y) const {
    const std::size_t i = std::size_t(y) * width_ + x;
    if (format_ == 0) return data_[i];
    if (format_ == 1) return (b5(data_ + 2 * i) * 255 + 15) / 31;
    return data_[3 * i + 2];
}

int Image::gray(int x, int y) const {
    if (format_ == 0) return data_[std::size_t(y) * width_ + x];
    // ITU-R BT.601 luma, integer arithmetic
    return (77 * red(x, y) + 150 * green(x, y) + 29 * blue(x, y) + 128) >> 8;
}

BallSample Ball::history(int k) const {
    if (k < 0 || k >= count_) return BallSample{x, y, time};
    return samples_[(head_ - k + kMaxHistory) % kMaxHistory];
}

PID::PID(float kp_, float ki_, float kd_) : kp(kp_), ki(ki_), kd(kd_) {}

void PID::reset() {
    integral_ = prevError_ = dFiltered_ = 0.0f;
    first_ = true;
}

void PID::setOutputLimit(float l) { outLimit_ = std::fabs(l); }
void PID::setIntegralLimit(float l) { intLimit_ = std::fabs(l); }
void PID::setDerivativeFilter(float s) { dTau_ = s > 0.0f ? s : 0.0f; }

float PID::update(float error, float dt) {
    if (dt <= 0.0f) dt = 1e-3f;
    integral_ += error * dt;
    if (intLimit_ > 0.0f) integral_ = clamp(integral_, -intLimit_, intLimit_);
    float d = first_ ? 0.0f : (error - prevError_) / dt;
    if (dTau_ > 0.0f && !first_) {
        const float a = dt / (dTau_ + dt);
        dFiltered_ += a * (d - dFiltered_);
        d = dFiltered_;
    } else {
        dFiltered_ = d;
    }
    first_ = false;
    prevError_ = error;
    lastP = kp * error;
    lastI = ki * integral_;
    lastD = kd * d;
    float out = lastP + lastI + lastD;
    if (outLimit_ > 0.0f) out = clamp(out, -outLimit_, outLimit_);
    return out;
}

void log(const char* name, float value) { detail::logValue(name, value); }

float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace bbp
