// The framework runtime: sits between the hardware (here: the simulator) and the student's
// track()/control(). It applies the Setup-tab effects (sensor delay, FPS cap, motor delay,
// position noise, modelled FireBeetle run time), converts pixels to plate millimetres,
// keeps the ball history, runs the inverse kinematics and drives the servos.
#pragma once

#include <deque>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "bbp/bbp.hpp"
#include "bbp/sim/rng.hpp"
#include "bbp/sim/world.hpp"
#include "settings.hpp"

namespace bbp::fw {

struct Telemetry {
    bool measured = false;
    double measX = 0, measY = 0;      // [m] last measured position
    double targetX = 0, targetY = 0;  // [m]
    double cmdAngleX = 0, cmdAngleY = 0;  // [deg] last commanded (after limits)
    double cpuTime = 0;               // [s] modelled FireBeetle time of the last cycle
    double hostTime = 0;              // [s] PC time of the last cycle
};

class Runtime {
public:
    Runtime(const Settings& s, sim::SimWorld& world);

    // Call after every simulator step.
    void update();

    void setTargetOverride(double x, double y);  // [m] switches to a fixed live target
    const Telemetry& telemetry() const { return tele_; }

    // Called for every frame handed to track(), with the student's detection.
    std::function<void(const sim::Frame&, const Detection&)> onFrame;
    std::function<void(const std::string& name, double t, double value)> onLog;
    std::function<void(int level, const std::string& text)> onText;  // 0 info, 1 warning, 2 error

private:
    struct PendingFrame {
        sim::Frame frame;
        double deliverAt;
    };
    struct PendingCommand {
        double applyAt;
        double pulse[3];
    };

    void process(sim::Frame& f, double t);
    void targetAt(double t, double& x, double& y) const;

    Settings s_;
    sim::SimWorld& world_;
    sim::Rng rng_;
    std::deque<PendingFrame> frames_;
    std::deque<PendingCommand> commands_;
    double lastAccepted_ = -1e9;
    double lastControl_ = -1.0;
    Ball ball_;
    bool haveVelocity_ = false;
    Pose<double> commandedPose_;
    double neutral_;
    bool overrideTarget_ = false;
    double overrideX_ = 0, overrideY_ = 0;
    bool reportedError_ = false, reportedLost_ = false, reportedReach_ = false;
    std::vector<std::pair<std::string, float>> logBuffer_;
    Telemetry tele_;
};

}  // namespace bbp::fw
