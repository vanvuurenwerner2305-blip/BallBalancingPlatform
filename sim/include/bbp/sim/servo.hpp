// Hobby position servo: the PWM command is latched once per servo frame, an internal
// proportional controller with a deadband drives a DC gear motor whose output torque follows
// the linear torque-speed characteristic of a permanent-magnet DC motor.
#pragma once

#include "bbp/sim/params.hpp"
#include "bbp/sim/rng.hpp"

namespace bbp::sim {

class ServoModel {
public:
    ServoModel(const ServoParams& p, std::uint64_t seed, int index);

    // Request a pulse width; it is quantised to the PWM generator resolution and takes effect
    // at the next servo frame (after the pulse has been fully received).
    void setPulse(double pulseUs);

    // Advance the discrete parts (frame latching, controller sampling) to time t, given the
    // current crank angle. Call once per physics step before integrating.
    void update(double t, double theta);

    // Output torque on the crank at the held drive level.
    double torque(double thetaDot) const;

    void reset(double t, double theta);

    double pulseToCrankAngle(double pulseUs) const;
    double crankAngleToPulse(double theta) const;

    double requestedPulse() const { return pending_; }
    double targetAngle() const { return target_; }
    double drive() const { return duty_; }
    bool hasTarget() const { return hasTarget_; }
    const ServoParams& params() const { return p_; }

private:
    ServoParams p_;
    Rng rng_;
    double pending_ = 1500.0;
    bool hasPending_ = false;
    double latched_ = 1500.0;
    double latchTime_ = 0.0;     // when the latched pulse becomes the target
    bool latchPending_ = false;
    double target_ = 0.0;
    bool hasTarget_ = false;
    double nextFrame_ = 0.0;
    double nextControl_ = 0.0;
    double duty_ = 0.0;
};

}  // namespace bbp::sim
