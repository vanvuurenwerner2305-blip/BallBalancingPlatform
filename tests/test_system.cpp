// Whole-system checks on SimWorld (servos + mechanism + ball, camera optional).
#include <algorithm>

#include "bbp/sim/world.hpp"
#include "test.hpp"

using namespace bbp;
using namespace bbp::sim;

namespace {
const double kDeg = 3.14159265358979323846 / 180.0;

SimParams fastParams() {
    SimParams p;
    p.camera.fps = 1.0;  // the camera is not needed here; keep rendering out of the way
    p.camera.spatialSamples = 1;
    p.camera.temporalSamples = 1;
    return p;
}
}  // namespace

TEST(System_BallStaysPutOnLevelPlate) {
    SimWorld w(fastParams());
    w.reset(0.0, 0.0);
    w.advance(1.0);
    const GroundTruth g = w.truth();
    std::printf("    after 1 s: ball (%.3g, %.3g) mm, mode %s, slopes (%.3g, %.3g) deg\n", g.ballPlate.x * 1e3,
                g.ballPlate.y * 1e3, toString(g.mode), g.slopeX / kDeg, g.slopeY / kDeg);
    CHECK(std::hypot(g.ballPlate.x, g.ballPlate.y) < 1e-3);
    CHECK(g.mode == ContactMode::Stuck || g.mode == ContactMode::Rolling);
}

TEST(System_ServoStepResponse) {
    SimWorld w(fastParams());
    w.reset(0.0, 0.0);
    const double step = 10.0 * kDeg;
    w.setServoPulse(0, w.pulseForCrankAngle(0, step));
    bbptest::Csv csv("servo_step.csv", "t,theta_deg,target_deg,omega");
    double maxRate = 0.0;
    for (int i = 0; i < 1600; ++i) {  // 0.4 s
        w.step();
        const GroundTruth g = w.truth();
        maxRate = std::max(maxRate, std::fabs(g.thetaDot[0]));
        csv.row(g.t, g.theta[0] / kDeg, g.targetTheta[0] / kDeg, g.thetaDot[0]);
    }
    const GroundTruth g = w.truth();
    const double halfDb = 0.5 * w.params().servo.deadbandUs /
                          ((w.params().servo.pulseMaxUs - w.params().servo.pulseMinUs) / 180.0);  // deg
    bbptest::report("peak crank speed [rad/s]", maxRate, w.params().servo.noLoadSpeed);
    std::printf("    final error %.3f deg (deadband +-%.3f deg, PWM quantisation incl.)\n",
                (g.targetTheta[0] - g.theta[0]) / kDeg, halfDb);
    CHECK(maxRate < w.params().servo.noLoadSpeed);
    CHECK(std::fabs(g.targetTheta[0] - g.theta[0]) / kDeg < halfDb + 0.5);
}

TEST(System_TiltedPlateBallRollsDownhill) {
    // Command a 3 deg slope about y via IK and check the ball accelerates at roughly
    // (5/7) g sin(3 deg) once the plate has settled.
    SimParams p = fastParams();
    p.contact.rollingResistance = 0.0;
    SimWorld w(p);
    w.reset(0.0, 0.0);
    double th[3];
    CHECK(w.kinematics().inverseSlopes(3.0 * kDeg, 0.0, w.neutralHeight(), th));
    for (int i = 0; i < 3; ++i) w.setServoPulse(i, w.pulseForCrankAngle(i, th[i]));
    w.advance(0.5);
    const GroundTruth g0 = w.truth();
    w.advance(0.2);
    const GroundTruth g1 = w.truth();
    const double aMeas = (g1.ballVelPlate.x - g0.ballVelPlate.x) / 0.2;
    const double aExp = -9.80665 * std::sin(g1.slopeX) / 1.4;
    std::printf("    settled slope %.3f deg (commanded 3), mode %s\n", g1.slopeX / kDeg, toString(g1.mode));
    bbptest::report("ball acceleration along x [m/s^2]", aMeas, aExp);
    CHECK_NEAR(aMeas, aExp, 0.03 * std::fabs(aExp));
}

TEST(System_ClosedLoopOnGroundTruthBalances) {
    // A PD controller on the true ball state must centre the ball: checks that the coupled
    // servo-mechanism-ball model is well posed and stable.
    SimWorld w(fastParams());
    w.reset(0.05, -0.03);
    const double g = 9.80665, k = 0.4;
    const double wn = 2.0 * 3.14159265358979323846 * 0.6, zeta = 0.8;
    for (int i = 0; i < 20000; ++i) {  // 5 s
        if (i % 80 == 0) {             // 50 Hz control
            const GroundTruth s = w.truth();
            const double ax = -(wn * wn * s.ballPlate.x + 2 * zeta * wn * s.ballVelPlate.x);
            const double ay = -(wn * wn * s.ballPlate.y + 2 * zeta * wn * s.ballVelPlate.y);
            // a = -(g/(1+k)) sin(slope)  ->  slope = asin(-a (1+k)/g)
            auto slope = [&](double a) { return std::asin(std::clamp(-a * (1 + k) / g, -0.15, 0.15)); };
            double th[3];
            if (w.kinematics().inverseSlopes(slope(ax), slope(ay), w.neutralHeight(), th))
                for (int j = 0; j < 3; ++j) w.setServoPulse(j, w.pulseForCrankAngle(j, th[j]));
        }
        w.step();
    }
    const GroundTruth s = w.truth();
    std::printf("    after 5 s: ball at (%.2f, %.2f) mm, mode %s\n", s.ballPlate.x * 1e3, s.ballPlate.y * 1e3,
                toString(s.mode));
    CHECK(std::hypot(s.ballPlate.x, s.ballPlate.y) < 0.003);
}
