// Verification of the ball-plate contact dynamics against classical closed-form results,
// using prescribed plate motions.
#include "bbp/sim/ball.hpp"
#include "test.hpp"

using namespace bbp;
using namespace bbp::sim;

namespace {
const double kDeg = 3.14159265358979323846 / 180.0;

PlateMotion staticPlate(const Mat3d& R) {
    PlateMotion pm;
    pm.pose.R = R;
    pm.pose.p = Vec3d{0, 0, 0};
    return pm;
}

// Ideal, lossless setup for the closed-form comparisons.
struct Setup {
    BallParams ball;
    ContactParams contact;
    PlateParams plate;
    EnvironmentParams env;
    Setup() {
        contact.rollingResistance = 0.0;
        env.airDensity = 0.0;
    }
    BallModel model(double plateRadius = 10.0) const { return BallModel(ball, contact, plate, env, plateRadius); }
};
}  // namespace

TEST(Ball_InclineAccelerationIncludesRollingInertia) {
    // Rolling without slipping down a fixed incline: a = g sin(beta) / (1 + k).
    for (double inner : {0.0, 0.009}) {
        Setup s;
        s.ball.innerRadius = inner;
        const BallModel m = s.model();
        const double beta = 5.0 * kDeg;
        const Mat3d R = rotationY(beta);  // normal tilted towards +x: downhill is +x
        BallIntegrator integ(m, [&](double) { return staticPlate(R); }, 1e-4);
        BallState b = m.restingOn(staticPlate(R), 0, 0);
        b.mode = ContactMode::Rolling;
        const Vec3d r0 = b.r;
        double t = 0.0;
        bbptest::Csv csv(inner == 0.0 ? "ball_incline_solid.csv" : "ball_incline_hollow.csv", "t,s,s_analytic,mode");
        const double aExp = m.gravity() * std::sin(beta) / (1.0 + m.inertiaFactor());
        while (t < 0.5 - 1e-12) {
            integ.step(b, t);
            const double sdist = (b.r - r0).norm();
            csv.row(t, sdist, 0.5 * aExp * t * t, int(b.mode));
        }
        const double sdist = (b.r - r0).norm();
        const double aMeas = 2.0 * sdist / (t * t);
        std::printf("    k = %.4f (%s)\n", m.inertiaFactor(), inner == 0.0 ? "solid" : "hollow");
        bbptest::report("acceleration down the 5 deg incline [m/s^2]", aMeas, aExp);
        CHECK(b.mode == ContactMode::Rolling);
        CHECK_NEAR(aMeas, aExp, 1e-6 * aExp);
        CHECK(b.r.x > r0.x);
    }
}

TEST(Ball_EnergyConservedWithoutDissipation) {
    Setup s;
    s.contact.staticFriction = 10.0;  // never slips
    const BallModel m = s.model();
    const Mat3d R = rotationZ(0.3) * rotationY(8.0 * kDeg) * rotationZ(-0.3);
    BallIntegrator integ(m, [&](double) { return staticPlate(R); }, 2.5e-4);
    BallState b = m.restingOn(staticPlate(R), 0.02, -0.01);
    b.v += Vec3d{0.1, 0.25, 0.0};  // give it a sideways push (then re-project)
    m.postStep(b, staticPlate(R));
    const double E0 = m.energy(b);
    double t = 0.0, worst = 0.0;
    while (t < 1.0) {
        integ.step(b, t);
        worst = std::max(worst, std::fabs(m.energy(b) - E0) / (m.mass() * m.gravity() * 0.01));
    }
    std::printf("    max |E - E0| / (m g 1cm) over 1 s = %.2e\n", worst);
    CHECK(worst < 1e-8);
}

TEST(Ball_SlidingTurnsIntoRollingAtOneOverOnePlusK) {
    // A ball launched sliding without spin on a level plate rolls after t* = k v0 / ((1+k) mu g)
    // with v* = v0 / (1 + k)  (5/7 v0 for a solid sphere).
    Setup s;
    const BallModel m = s.model();
    const Mat3d R = Mat3d::identity();
    BallIntegrator integ(m, [&](double) { return staticPlate(R); }, 1e-4);
    BallState b = m.restingOn(staticPlate(R), 0, 0);
    const double v0 = 0.3;
    b.v = Vec3d{v0, 0, 0};
    b.w = Vec3d{};
    b.mode = ContactMode::Slipping;
    b.slipDir = Vec3d{1, 0, 0};
    double t = 0.0, tRoll = -1.0;
    bbptest::Csv csv("ball_slide.csv", "t,v,a_omega,mode");
    while (t < 0.1) {
        integ.step(b, t);
        csv.row(t, b.v.x, m.radius() * b.w.y, int(b.mode));
        if (tRoll < 0 && b.mode == ContactMode::Rolling) tRoll = t;
    }
    const double k = m.inertiaFactor();
    const double tExp = k * v0 / ((1 + k) * s.contact.kineticFriction * m.gravity());
    bbptest::report("time to pure rolling [s]", tRoll, tExp);
    bbptest::report("rolling speed [m/s]", b.v.x, v0 / (1 + k));
    CHECK_NEAR(tRoll, tExp, 1.5e-4);
    CHECK_NEAR(b.v.x, v0 / (1 + k), 1e-6);
}

TEST(Ball_RollingResistanceDeceleratesThenSticks) {
    // Constant resisting couple c_rr a N: deceleration c_rr g / (1 + k), then static hold.
    Setup s;
    s.contact.rollingResistance = 0.01;
    const BallModel m = s.model();
    const Mat3d R = Mat3d::identity();
    BallIntegrator integ(m, [&](double) { return staticPlate(R); }, 2.5e-4);
    BallState b = m.restingOn(staticPlate(R), 0, 0);
    b.v = Vec3d{0.2, 0, 0};
    b.w = Vec3d{0, 0.2 / m.radius(), 0};  // rolling: v = a w x n
    m.postStep(b, staticPlate(R));
    CHECK_NEAR(b.v.x, 0.2, 1e-12);
    double t = 0.0;
    double v1 = 0.0;
    bbptest::Csv csv("ball_rolling_resistance.csv", "t,v,mode");
    while (t < 4.0) {
        integ.step(b, t);
        if (std::fabs(t - 1.0) < 1e-9) v1 = b.v.x;
        csv.row(t, b.v.x, int(b.mode));
    }
    const double dec = s.contact.rollingResistance * m.gravity() / (1 + m.inertiaFactor());
    bbptest::report("deceleration [m/s^2]", 0.2 - v1, dec);
    CHECK_NEAR(0.2 - v1, dec, 1e-6);
    CHECK(b.mode == ContactMode::Stuck);
    CHECK_NEAR(b.v.x, 0.0, 1e-12);
}

TEST(Ball_TurntableOrbitAtTwoSeventhsOfPlateRate) {
    // Classic result (Weckesser 1997): on a turntable spinning at Omega a rolling ball's centre
    // moves on a circle at k Omega / (1 + k) = 2/7 Omega for a solid sphere.
    Setup s;
    s.contact.staticFriction = 10.0;
    const BallModel m = s.model(10.0);
    const double Om = 2.0 * 3.14159265358979323846;  // 1 rev/s
    auto motion = [&](double t) {
        PlateMotion pm;
        pm.pose.R = rotationZ(Om * t);
        pm.w = Vec3d{0, 0, Om};
        return pm;
    };
    BallIntegrator integ(m, motion, 2.5e-4);
    BallState b = m.restingOn(motion(0), 0.05, 0.0);
    b.v += Vec3d{0.0, -0.1, 0.0};  // not co-rotating: give it a kick
    m.postStep(b, motion(0));
    const Vec3d r0 = b.r;
    const double k = m.inertiaFactor();
    const double OmBall = k * Om / (1 + k);
    const double T = 2.0 * 3.14159265358979323846 / OmBall;  // one orbit = 3.5 turntable revs
    double t = 0.0;
    double vAng0 = std::atan2(b.v.y, b.v.x), vAng = vAng0, unwrapped = 0.0;
    bbptest::Csv csv("ball_turntable.csv", "t,x,y");
    while (t < T - 1e-9) {
        integ.step(b, t);
        const double a = std::atan2(b.v.y, b.v.x);
        double d = a - vAng;
        if (d > 3.14159265358979323846) d -= 2 * 3.14159265358979323846;
        if (d < -3.14159265358979323846) d += 2 * 3.14159265358979323846;
        unwrapped += d;
        vAng = a;
        csv.row(t, b.r.x, b.r.y);
    }
    bbptest::report("orbit rate / turntable rate", unwrapped / t / Om, k / (1 + k));
    bbptest::report("closure error after one orbit [m]", (b.r - r0).norm(), 0.0);
    CHECK_NEAR(unwrapped / t / Om, k / (1 + k), 1e-6);
    CHECK((b.r - r0).norm() < 1e-6);
}

TEST(Ball_NormalForceAndSeparationOnAcceleratingPlate) {
    Setup s;
    const BallModel m = s.model();
    for (double az : {0.5, -0.5, -1.5}) {
        const double a = az * m.gravity();
        auto motion = [&](double t) {
            PlateMotion pm;
            pm.pose.p = Vec3d{0, 0, 0.5 * a * t * t};
            pm.v = Vec3d{0, 0, a * t};
            pm.a = Vec3d{0, 0, a};
            return pm;
        };
        BallState b = m.restingOn(motion(0), 0, 0);
        const BallRates r = m.rates(b, motion(0));
        BallIntegrator integ(m, motion, 1e-4);
        double t = 0.0;
        BallEvents ev;
        integ.step(b, t, &ev);
        if (az > -1.0) {
            bbptest::report("normal force [N]", r.normalForce, m.mass() * (m.gravity() + a));
            CHECK_NEAR(r.normalForce, m.mass() * (m.gravity() + a), 1e-12);
            CHECK(b.mode != ContactMode::Flight);
        } else {
            std::printf("    plate accelerating down at 1.5 g: mode = %s\n", toString(b.mode));
            CHECK(b.mode == ContactMode::Flight);
        }
    }
}

TEST(Ball_PivotingFrictionStopsSpin) {
    // Spin about the normal decays at M_p / I with M_p = (3 pi / 16) mu N a_c (Hertz contact).
    Setup s;
    const BallModel m = s.model();
    const Mat3d R = Mat3d::identity();
    BallIntegrator integ(m, [&](double) { return staticPlate(R); }, 2.5e-4);
    BallState b = m.restingOn(staticPlate(R), 0, 0);
    b.w = Vec3d{0, 0, 20.0};
    b.spinStuck = false;
    b.spinSign = 1.0;
    const double N = m.mass() * m.gravity();
    const double rate = m.pivotingCouple(N) / m.inertia();
    double t = 0.0;
    for (int i = 0; i < 2000; ++i) integ.step(b, t);
    std::printf("    Hertz contact radius %.1f um, pivoting couple %.3e N m\n", m.hertzRadius(N) * 1e6,
                m.pivotingCouple(N));
    bbptest::report("spin after 0.5 s [rad/s]", b.w.z, std::max(0.0, 20.0 - rate * 0.5));
    CHECK_NEAR(b.w.z, std::max(0.0, 20.0 - rate * 0.5), 1e-6);
}

TEST(Ball_BounceUsesRestitution) {
    Setup s;
    const BallModel m = s.model();
    const Mat3d R = Mat3d::identity();
    BallIntegrator integ(m, [&](double) { return staticPlate(R); }, 1e-5);
    BallState b = m.restingOn(staticPlate(R), 0, 0);
    const double h = 0.01;
    b.r.z += h;
    b.mode = ContactMode::Flight;
    double t = 0.0;
    BallEvents ev;
    while (!ev.impact) integ.step(b, t, &ev);
    const double vImpact = std::sqrt(2 * m.gravity() * h);
    bbptest::report("impact speed [m/s]", ev.impactSpeed, vImpact);
    bbptest::report("rebound speed [m/s]", b.v.z, s.contact.restitution * vImpact);
    CHECK_NEAR(ev.impactSpeed, vImpact, 2e-4);
    CHECK_NEAR(b.v.z, s.contact.restitution * ev.impactSpeed, 1e-12);
    CHECK(b.mode == ContactMode::Flight);
}
