#include "bbp/sim/world.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bbp::sim {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
}

SimWorld::SimWorld(const SimParams& p)
    : p_(p),
      dyn_(p),
      ball_(p.ball, p.contact, p.plate, p.env, p.geometry.plateRadius),
      renderer_(std::make_unique<CameraRenderer>(p)),
      servos_{ServoModel(p.servo, p.numerics.seed, 0), ServoModel(p.servo, p.numerics.seed, 1),
              ServoModel(p.servo, p.numerics.seed, 2)} {
    reset();
}

double SimWorld::neutralHeight() const {
    return kinematics().levelHeight(p_.servo.centreCrankAngleDeg * kDeg);
}

void SimWorld::reset(double ballX, double ballY) {
    t_ = 0.0;
    const double th0 = p_.servo.centreCrankAngleDeg * kDeg;
    for (int i = 0; i < 3; ++i) {
        th_[i] = th0;
        thd_[i] = 0.0;
        servos_[i].reset(0.0, th0);
        servos_[i].setPulse(servos_[i].crankAngleToPulse(th0));
    }
    pose_ = kinematics().poseFromNormal(Vec3d{0, 0, 1}, neutralHeight());
    if (kinematics().forward(th_, pose_) < 0) throw std::runtime_error("forward kinematics failed at reset");
    ballForce_ = ballPoint_ = ballCouple_ = Vec3d{};
    State x{{th_[0], th_[1], th_[2]}, {0, 0, 0}, Vec3d{}, Vec3d{}, Vec3d{}};
    Rates d;
    if (!rates(x, d, &pm_)) throw std::runtime_error("mechanism evaluation failed at reset");
    bs_ = ball_.restingOn(pm_, ballX, ballY);
    const BallRates br = ball_.rates(bs_, pm_);
    ballForce_ = br.force * -1.0;
    ballPoint_ = bs_.r - pm_.pose.normal() * ball_.radius();
    ballCouple_ = br.moment * -1.0;
    history_.clear();
    frames_.clear();
    frameIndex_ = 0;
    nextFrameStart_ = 0.0;
    record();
}

void SimWorld::setServoPulse(int i, double us) { servos_[i].setPulse(us); }

void SimWorld::nudgeBall(double vx, double vy) {
    if (bs_.mode == ContactMode::Lost || bs_.mode == ContactMode::Flight) return;
    const Vec3d n = pose_.normal();
    const Vec3d dv = pose_.R * Vec3d{vx, vy, 0.0};
    bs_.v += dv;
    bs_.w += n.cross(dv) / ball_.radius();  // keep it rolling: dv = a dw x n
    bs_.mode = ContactMode::Rolling;
}

bool SimWorld::rates(const State& x, Rates& d, PlateMotion* pmOut) {
    double tau[3], thdd[3];
    for (int i = 0; i < 3; ++i) tau[i] = servos_[i].torque(x.thd[i]);
    Posed pose = pose_;
    PlateMotion pm;
    if (!dyn_.evaluate(x.th, x.thd, tau, ballForce_, ballPoint_, ballCouple_, pose, thdd, pm)) return false;
    for (int i = 0; i < 3; ++i) {
        d.dth[i] = x.thd[i];
        d.dthd[i] = thdd[i];
    }
    if (bs_.mode == ContactMode::Lost) {
        d.dr = d.dv = d.dw = Vec3d{};
    } else {
        BallState b = bs_;
        b.r = x.r;
        b.v = x.v;
        b.w = x.w;
        const BallRates br = ball_.rates(b, pm);
        d.dr = x.v;
        d.dv = br.dv;
        d.dw = br.dw;
    }
    if (pmOut) *pmOut = pm;
    return true;
}

void SimWorld::step() {
    const double h = p_.numerics.dt;
    for (int i = 0; i < 3; ++i) servos_[i].update(t_, th_[i]);

    State x0{{th_[0], th_[1], th_[2]}, {thd_[0], thd_[1], thd_[2]}, bs_.r, bs_.v, bs_.w};
    auto axpy = [](const State& x, const Rates& k, double s) {
        State y = x;
        for (int i = 0; i < 3; ++i) {
            y.th[i] += k.dth[i] * s;
            y.thd[i] += k.dthd[i] * s;
        }
        y.r += k.dr * s;
        y.v += k.dv * s;
        y.w += k.dw * s;
        return y;
    };
    Rates k1, k2, k3, k4;
    bool ok = rates(x0, k1, nullptr);
    ok = ok && rates(axpy(x0, k1, 0.5 * h), k2, nullptr);
    ok = ok && rates(axpy(x0, k2, 0.5 * h), k3, nullptr);
    ok = ok && rates(axpy(x0, k3, h), k4, nullptr);
    if (!ok) throw std::runtime_error("mechanism evaluation failed (singular configuration)");
    State xe = x0;
    for (int i = 0; i < 3; ++i) {
        xe.th[i] += h / 6.0 * (k1.dth[i] + 2 * k2.dth[i] + 2 * k3.dth[i] + k4.dth[i]);
        xe.thd[i] += h / 6.0 * (k1.dthd[i] + 2 * k2.dthd[i] + 2 * k3.dthd[i] + k4.dthd[i]);
    }
    xe.r += (k1.dr + k2.dr * 2.0 + k3.dr * 2.0 + k4.dr) * (h / 6.0);
    xe.v += (k1.dv + k2.dv * 2.0 + k3.dv * 2.0 + k4.dv) * (h / 6.0);
    xe.w += (k1.dw + k2.dw * 2.0 + k3.dw * 2.0 + k4.dw) * (h / 6.0);
    t_ += h;

    for (int i = 0; i < 3; ++i) {
        th_[i] = xe.th[i];
        thd_[i] = xe.thd[i];
    }
    Rates dummy;
    if (!rates(xe, dummy, &pm_)) throw std::runtime_error("mechanism evaluation failed (singular configuration)");
    pose_ = pm_.pose;

    bs_.r = xe.r;
    bs_.v = xe.v;
    bs_.w = xe.w;
    ball_.postStep(bs_, pm_);
    if (bs_.mode == ContactMode::Lost) bs_.v = bs_.w = Vec3d{};

    if (bs_.mode == ContactMode::Rolling || bs_.mode == ContactMode::Stuck || bs_.mode == ContactMode::Slipping) {
        const BallRates br = ball_.rates(bs_, pm_);
        ballForce_ = br.force * -1.0;
        ballPoint_ = bs_.r - pm_.pose.normal() * ball_.radius();
        ballCouple_ = br.moment * -1.0;
    } else {
        ballForce_ = ballCouple_ = Vec3d{};
    }
    record();
    serviceCamera();
}

void SimWorld::advance(double duration) {
    const double end = t_ + duration;
    while (t_ < end - 1e-12) step();
}

void SimWorld::record() {
    history_.push_back(Snapshot{t_, pose_.p, Quat<double>::fromMatrix(pose_.R), bs_.r, bs_.mode != ContactMode::Lost});
    while (history_.size() > 2 && history_.front().t < t_ - p_.numerics.historyWindow) history_.pop_front();
}

SceneState SimWorld::sceneAt(double t) const {
    SceneState s;
    auto fill = [&](const Snapshot& a) {
        s.plate.p = a.p;
        s.plate.R = a.q.toMatrix();
        s.ball = a.ball;
        s.ballPresent = a.ballPresent;
    };
    if (t <= history_.front().t) {
        fill(history_.front());
        return s;
    }
    if (t >= history_.back().t) {
        fill(history_.back());
        return s;
    }
    auto it = std::upper_bound(history_.begin(), history_.end(), t,
                               [](double v, const Snapshot& sn) { return v < sn.t; });
    const Snapshot& b = *it;
    const Snapshot& a = *(it - 1);
    const double u = (t - a.t) / (b.t - a.t);
    s.plate.p = a.p + (b.p - a.p) * u;
    s.plate.R = Quat<double>::nlerp(a.q, b.q, u).toMatrix();
    s.ball = a.ball + (b.ball - a.ball) * u;
    s.ballPresent = a.ballPresent && b.ballPresent;
    return s;
}

void SimWorld::serviceCamera() {
    while (renderer_->exposureEnd(nextFrameStart_) <= t_) {
        Frame f;
        renderer_->render([this](double tt) { return sceneAt(tt); }, nextFrameStart_, frameIndex_++, f);
        frames_.push_back(std::move(f));
        nextFrameStart_ += 1.0 / p_.camera.fps;
    }
}

bool SimWorld::pollFrame(Frame& out) {
    if (frames_.empty() || frames_.front().readyTime > t_) return false;
    out = std::move(frames_.front());
    frames_.pop_front();
    return true;
}

GroundTruth SimWorld::truth() const {
    GroundTruth g;
    g.t = t_;
    g.plate = pose_;
    const Vec3d n = pose_.normal();
    g.ballWorld = bs_.r;
    g.ballPlate = pose_.toPlate(bs_.r);
    g.ballVelPlate = pose_.R.transposed() * (bs_.v - pm_.velocityAt(bs_.r));
    g.mode = bs_.mode;
    g.slopeX = std::atan(-n.x / n.z);
    g.slopeY = std::atan(-n.y / n.z);
    for (int i = 0; i < 3; ++i) {
        g.theta[i] = th_[i];
        g.thetaDot[i] = thd_[i];
        g.targetTheta[i] = servos_[i].targetAngle();
    }
    if (bs_.mode != ContactMode::Flight && bs_.mode != ContactMode::Lost)
        g.normalForce = ball_.rates(bs_, pm_).normalForce;
    return g;
}

}  // namespace bbp::sim
