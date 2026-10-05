#include "runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>

#include "runtime_hooks.hpp"

namespace bbp::detail {
namespace {
LogSink& sink() {
    static LogSink s;
    return s;
}
}  // namespace
void setLogSink(LogSink s) { sink() = std::move(s); }
void logValue(const char* name, float value) {
    if (sink()) sink()(name, value);
}
}  // namespace bbp::detail

namespace bbp::fw {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
using Clock = std::chrono::steady_clock;
}  // namespace

Runtime::Runtime(const Settings& s, sim::SimWorld& world)
    : s_(s), world_(world), rng_(s.sim.numerics.seed, 0xf00d) {
    neutral_ = world_.neutralHeight();
    commandedPose_ = world_.kinematics().poseFromNormal(Vec3<double>{0, 0, 1}, neutral_);
    detail::setLogSink([this](const char* name, float v) { logBuffer_.emplace_back(name ? name : "?", v); });
}

void Runtime::setTargetOverride(double x, double y) {
    overrideTarget_ = true;
    overrideX_ = x;
    overrideY_ = y;
}

void Runtime::targetAt(double t, double& x, double& y) const {
    if (overrideTarget_) {
        x = overrideX_;
        y = overrideY_;
        return;
    }
    const auto& T = s_.target;
    const double w = 2.0 * kPi / std::max(T.period, 0.1);
    switch (T.mode) {
        case TargetMode::Fixed:
            x = T.x;
            y = T.y;
            break;
        case TargetMode::Circle:
            x = T.x + T.radius * std::cos(w * t);
            y = T.y + T.radius * std::sin(w * t);
            break;
        case TargetMode::Figure8:
            x = T.x + T.radius * std::sin(w * t);
            y = T.y + 0.5 * T.radius * std::sin(2.0 * w * t);
            break;
        case TargetMode::Square: {
            const double ph = std::fmod(t / std::max(T.period, 0.1), 1.0);
            const int corner = int(ph * 4.0);
            const double cx[4] = {1, -1, -1, 1}, cy[4] = {1, 1, -1, -1};
            x = T.x + T.radius * cx[corner];
            y = T.y + T.radius * cy[corner];
            break;
        }
    }
}

void Runtime::update() {
    const double t = world_.time();

    // Camera -> (FPS cap) -> (sensor delay) -> track()
    sim::Frame f;
    while (world_.pollFrame(f)) {
        if (s_.effects.fpsCap > 0.0 && f.exposureStart - lastAccepted_ < 1.0 / s_.effects.fpsCap - 1e-6) continue;
        lastAccepted_ = f.exposureStart;
        const double deliverAt = f.readyTime + s_.effects.sensorDelay;
        frames_.push_back(PendingFrame{std::move(f), deliverAt});
    }
    while (!frames_.empty() && frames_.front().deliverAt <= t) {
        PendingFrame pf = std::move(frames_.front());
        frames_.pop_front();
        process(pf.frame, t);
    }

    // (motor delay) -> servos
    while (!commands_.empty() && commands_.front().applyAt <= t) {
        for (int i = 0; i < 3; ++i) world_.setServoPulse(i, commands_.front().pulse[i]);
        commands_.pop_front();
    }

    if (!reportedLost_ && world_.truth().mode == sim::ContactMode::Lost) {
        reportedLost_ = true;
        if (onText) onText(1, "The ball fell off the plate. Press Restart to try again.");
    }
}

void Runtime::process(sim::Frame& f, double t) {
    const Image img(f.data.data(), f.width, f.height, int(f.format));
    Detection det;
    Platform out;
    double tx, ty;
    targetAt(t, tx, ty);
    Target target{float(tx * 1e3), float(ty * 1e3)};
    logBuffer_.clear();

    const auto c0 = Clock::now();
    bool ok = true;
    try {
        track(img, det);
    } catch (const std::exception& e) {
        ok = false;
        if (!reportedError_ && onText) onText(2, std::string("track() threw an exception: ") + e.what());
        reportedError_ = true;
    } catch (...) {
        ok = false;
        if (!reportedError_ && onText) onText(2, "track() threw an exception");
        reportedError_ = true;
    }
    const auto c1 = Clock::now();

    // Pixel -> plate millimetres (through the plate, at ball-centre height), plus noise.
    ball_.dt = lastControl_ < 0.0 ? 0.0f : float(t - lastControl_);
    ball_.time = float(t);
    lastControl_ = t;
    ball_.found = false;
    if (ok && det.found) {
        Vec3<double> xp;
        const auto& cam = world_.camera();
        if (cam.backprojectToPlate(det.x, det.y, commandedPose_, s_.sim.geometry.plateRadius, s_.sim.ball.radius, xp)) {
            const double nx = s_.effects.positionNoise * rng_.normal();
            const double ny = s_.effects.positionNoise * rng_.normal();
            const float mx = float((xp.x + nx) * 1e3), my = float((xp.y + ny) * 1e3);
            if (ball_.count_ > 0) {
                const BallSample prev = ball_.history(0);
                const float dtm = float(t) - prev.t;
                if (dtm > 1e-6f) {
                    const float a = float(std::clamp(s_.control.velocityFilter, 0.0, 0.99));
                    const float rvx = (mx - prev.x) / dtm, rvy = (my - prev.y) / dtm;
                    ball_.vx = haveVelocity_ ? a * ball_.vx + (1 - a) * rvx : rvx;
                    ball_.vy = haveVelocity_ ? a * ball_.vy + (1 - a) * rvy : rvy;
                    haveVelocity_ = true;
                }
            }
            ball_.x = mx;
            ball_.y = my;
            ball_.found = true;
            ball_.head_ = (ball_.head_ + 1) % Ball::kMaxHistory;
            ball_.samples_[ball_.head_] = BallSample{mx, my, float(t)};
            if (ball_.count_ < Ball::kMaxHistory) ++ball_.count_;
        }
    }

    const auto c2 = Clock::now();
    if (ok) {
        try {
            control(ball_, target, out);
        } catch (const std::exception& e) {
            ok = false;
            if (!reportedError_ && onText) onText(2, std::string("control() threw an exception: ") + e.what());
            reportedError_ = true;
        } catch (...) {
            ok = false;
            if (!reportedError_ && onText) onText(2, "control() threw an exception");
            reportedError_ = true;
        }
    }
    const auto c3 = Clock::now();

    const double host = std::chrono::duration<double>((c1 - c0) + (c3 - c2)).count();
    tele_.hostTime = host;
    tele_.cpuTime = s_.effects.cpuModel ? host * s_.effects.cpuFactor : 0.0;
    tele_.measured = ball_.found;
    tele_.measX = ball_.x * 1e-3;
    tele_.measY = ball_.y * 1e-3;
    tele_.targetX = tx;
    tele_.targetY = ty;

    if (onFrame) onFrame(f, det);
    if (onLog)
        for (const auto& [name, v] : logBuffer_) onLog(name, t, v);
    if (!ok) return;

    // Limits, inverse kinematics, servo pulses (after modelled run time + motor delay).
    const double lim = s_.control.maxAngleDeg;
    auto finite = [](float v) { return std::isfinite(v) ? double(v) : 0.0; };
    const double ax = std::clamp(finite(out.angleX), -lim, lim);
    const double ay = std::clamp(finite(out.angleY), -lim, lim);
    tele_.cmdAngleX = ax;
    tele_.cmdAngleY = ay;
    double th[3];
    Pose<double> pose;
    const auto& kin = world_.kinematics();
    if (!kin.inverse(Kinematics<double>::normalFromSlopes(ax * kDeg, ay * kDeg), neutral_, th, &pose)) {
        if (!reportedReach_ && onText) onText(1, "Requested plate angle is out of reach; command ignored.");
        reportedReach_ = true;
        return;
    }
    commandedPose_ = pose;
    PendingCommand cmd;
    cmd.applyAt = t + tele_.cpuTime + s_.effects.motorDelay;
    for (int i = 0; i < 3; ++i) cmd.pulse[i] = world_.pulseForCrankAngle(i, th[i]);
    commands_.push_back(cmd);
}

}  // namespace bbp::fw
