#include "bbp/sim/servo.hpp"

#include <algorithm>
#include <cmath>

namespace bbp::sim {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
}

ServoModel::ServoModel(const ServoParams& p, std::uint64_t seed, int index)
    : p_(p), rng_(seed, 0x5e7000ULL + std::uint64_t(index)) {}

void ServoModel::reset(double t, double theta) {
    nextFrame_ = t;
    nextControl_ = t;
    duty_ = 0.0;
    // A powered servo that is already holding its current position.
    target_ = theta;
    hasTarget_ = true;
    latchPending_ = false;
}

double ServoModel::pulseToCrankAngle(double us) const {
    const double centre = 0.5 * (p_.pulseMinUs + p_.pulseMaxUs);
    const double servoAngle = (us - centre) / (p_.pulseMaxUs - p_.pulseMinUs) * 180.0 * kDeg;
    return p_.centreCrankAngleDeg * kDeg + p_.direction * servoAngle;
}

double ServoModel::crankAngleToPulse(double theta) const {
    const double centre = 0.5 * (p_.pulseMinUs + p_.pulseMaxUs);
    const double servoAngle = (theta - p_.centreCrankAngleDeg * kDeg) * p_.direction;
    return centre + servoAngle / (180.0 * kDeg) * (p_.pulseMaxUs - p_.pulseMinUs);
}

void ServoModel::setPulse(double us) {
    us = std::clamp(us, p_.pulseMinUs, p_.pulseMaxUs);
    pending_ = std::round(us / p_.pulseResolutionUs) * p_.pulseResolutionUs;
    hasPending_ = true;
}

void ServoModel::update(double t, double theta) {
    // Servo frames: the pulse starts at the frame boundary and is known to the servo once it
    // has ended, pulse-width later.
    while (t >= nextFrame_) {
        if (hasPending_) {
            latched_ = pending_;
            latchTime_ = nextFrame_ + latched_ * 1e-6;
            latchPending_ = true;
        }
        nextFrame_ += p_.pwmPeriod;
    }
    if (latchPending_ && t >= latchTime_) {
        target_ = pulseToCrankAngle(latched_);
        hasTarget_ = true;
        latchPending_ = false;
    }
    // Internal controller, sampled at its own rate with a zero-order hold on the drive.
    const double Tc = 1.0 / p_.controlRate;
    while (t >= nextControl_) {
        nextControl_ += Tc;
        if (!hasTarget_) {
            duty_ = 0.0;
            continue;
        }
        const double measured = theta + p_.potNoiseDeg * kDeg * rng_.normal();
        const double err = target_ - measured;
        const double usPerRad = (p_.pulseMaxUs - p_.pulseMinUs) / (180.0 * kDeg);
        const double halfDeadband = 0.5 * p_.deadbandUs / usPerRad;
        if (std::fabs(err) < halfDeadband) {
            duty_ = 0.0;
        } else {
            duty_ = std::clamp(err / (p_.proportionalBand * kDeg), -1.0, 1.0);
        }
    }
}

double ServoModel::torque(double thetaDot) const {
    // Linear torque-speed line of a PM DC motor at drive level u (duty * supply):
    // tau = tau_stall * u - (tau_stall / omega_0) * omega. At u = 0 the bridge brakes the motor.
    const double kv = p_.stallTorque / p_.noLoadSpeed;
    const double friction = p_.coulombFriction * std::tanh(thetaDot / 0.01);
    return p_.stallTorque * duty_ - kv * thetaDot - friction;
}

}  // namespace bbp::sim
