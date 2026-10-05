// The complete simulated platform: three servos driving the 3-RRS mechanism, the ball on the
// plate and the camera underneath, advanced on one fixed-step timeline. This is the object the
// simulator's hardware-abstraction layer talks to: servo pulses in, camera frames out.
#pragma once

#include <deque>
#include <memory>

#include "bbp/sim/ball.hpp"
#include "bbp/sim/platform_dynamics.hpp"
#include "bbp/sim/renderer.hpp"
#include "bbp/sim/servo.hpp"

namespace bbp::sim {

struct GroundTruth {
    double t = 0.0;
    Vec3d ballPlate;      // ball centre in plate coordinates [m]
    Vec3d ballVelPlate;   // ball-centre velocity relative to the plate, plate coordinates [m/s]
    Vec3d ballWorld;
    ContactMode mode = ContactMode::Rolling;
    Posed plate;
    double slopeX = 0.0, slopeY = 0.0;  // plate surface slope angles [rad]
    double theta[3]{}, thetaDot[3]{};   // crank angles and rates [rad, rad/s]
    double targetTheta[3]{};            // servo targets [rad]
    double normalForce = 0.0;           // [N]
};

class SimWorld {
public:
    explicit SimWorld(const SimParams& p = SimParams{});

    // Plate level at the given crank angle (default: the servo centre), ball at rest at
    // plate coordinates (x, y) [m], all servos commanded to hold.
    void reset(double ballX = 0.0, double ballY = 0.0);

    void setServoPulse(int servo, double pulseUs);
    // Disturbance: add a velocity [m/s] along the plate's x/y to the ball (rolling).
    void nudgeBall(double vx, double vy);
    void step();
    void advance(double duration);
    double time() const { return t_; }

    // Returns the next frame whose ready time has passed.
    bool pollFrame(Frame& out);

    GroundTruth truth() const;
    const SimParams& params() const { return p_; }
    const Kinematics<double>& kinematics() const { return dyn_.kinematics(); }
    const PlatformDynamics& dynamics() const { return dyn_; }
    const BallModel& ballModel() const { return ball_; }
    const CameraRenderer& renderer() const { return *renderer_; }
    const CameraGeometry<double>& camera() const { return renderer_->geometry(); }
    const ServoModel& servo(int i) const { return servos_[i]; }
    double neutralHeight() const;
    // Pulse width that commands crank i to angle theta (inverse of the servo's mapping).
    double pulseForCrankAngle(int i, double theta) const { return servos_[i].crankAngleToPulse(theta); }

    // Scene (plate pose, ball) interpolated from the recorded history.
    SceneState sceneAt(double t) const;

private:
    struct Snapshot {
        double t;
        Vec3d p;
        Quat<double> q;
        Vec3d ball;
        bool ballPresent;
    };
    struct State {
        double th[3], thd[3];
        Vec3d r, v, w;
    };
    struct Rates {
        double dth[3], dthd[3];
        Vec3d dr, dv, dw;
    };

    bool rates(const State& x, Rates& d, PlateMotion* pmOut);
    void record();
    void serviceCamera();

    SimParams p_;
    PlatformDynamics dyn_;
    BallModel ball_;
    std::unique_ptr<CameraRenderer> renderer_;
    ServoModel servos_[3];

    double t_ = 0.0;
    double th_[3]{}, thd_[3]{};
    BallState bs_;
    Posed pose_;          // forward-kinematics warm start
    PlateMotion pm_;      // plate motion at the current time
    // Contact wrench exerted by the ball on the plate, lagged by one step.
    Vec3d ballForce_, ballPoint_, ballCouple_;

    std::deque<Snapshot> history_;
    std::deque<Frame> frames_;
    std::uint64_t frameIndex_ = 0;
    double nextFrameStart_ = 0.0;
};

}  // namespace bbp::sim
