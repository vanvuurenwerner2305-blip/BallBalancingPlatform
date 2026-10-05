// Verification of the 3-RRS kinematics.
#include "bbp/core/kinematics.hpp"
#include "test.hpp"

using namespace bbp;
using V = Vec3<double>;
using M = Mat3<double>;

namespace {
const double kDeg = kPi<double> / 180.0;

// Rotation vector of a rotation matrix (inverse Rodrigues), valid away from 180 degrees.
V rotationVector(const M& R) {
    const double c = std::max(-1.0, std::min(1.0, 0.5 * (R.m[0][0] + R.m[1][1] + R.m[2][2] - 1.0)));
    const double ang = std::acos(c);
    const V axis{R.m[2][1] - R.m[1][2], R.m[0][2] - R.m[2][0], R.m[1][0] - R.m[0][1]};
    if (ang < 1e-12) return axis * 0.5;
    return axis * (ang / (2.0 * std::sin(ang)));
}

// Torsion of R relative to the zero-torsion rotation with the same normal.
double torsion(const M& R) {
    const V n = R.col(2);
    const double tilt = std::acos(std::min(1.0, n.z));
    const double phi = std::atan2(n.y, n.x);
    const M R0 = rotationZ(phi) * rotationY(tilt) * rotationZ(-phi);
    return rotationVector(R * R0.transposed()).dot(n);
}
}  // namespace

TEST(Kinematics_CadPoseReproducesCadAssembly) {
    // With every crank at the CAD angle the forward kinematics must return the CAD plate pose:
    // level, top face at kPlateTopZ. The residual is the effect of idealising the CAD's leg
    // planes (0.77 mm off the central axis) to pass exactly through it.
    const Kinematics<double> kin(PlatformGeometry<double>::fromCad());
    const double th = cad::kCrankAngleInCadDeg * kDeg;
    double theta[3] = {th, th, th};
    Pose<double> pose = kin.poseFromNormal(V{0, 0, 1}, 0.0);
    CHECK(kin.forward(theta, pose) >= 0);
    bbptest::report("plate-top z in CAD pose [mm]", pose.p.z * 1e3, cad::kPlateTopZ);
    CHECK_NEAR(pose.p.z, cad::kPlateTopZ * 1e-3, 1e-5);
    CHECK_NEAR(pose.normal().z, 1.0, 1e-12);
    CHECK_NEAR(std::hypot(pose.p.x, pose.p.y), 0.0, 1e-9);
}

TEST(Kinematics_InverseForwardRoundTrip) {
    const Kinematics<double> kin(PlatformGeometry<double>::fromCad());
    const double h0 = kin.levelHeight(0.0);
    double worstPos = 0.0, worstAng = 0.0, worstTorsion = 0.0, worstPlane = 0.0;
    int n = 0;
    for (double dh : {-0.010, 0.0, 0.010}) {
        for (double tilt = 0.0; tilt <= 15.0 + 1e-9; tilt += 2.5) {
            for (double az = 0.0; az < 360.0; az += 15.0) {
                const V nrm{std::sin(tilt * kDeg) * std::cos(az * kDeg), std::sin(tilt * kDeg) * std::sin(az * kDeg),
                            std::cos(tilt * kDeg)};
                double theta[3];
                Pose<double> target;
                CHECK(kin.inverse(nrm, h0 + dh, theta, &target));
                Pose<double> pose = kin.poseFromNormal(V{0, 0, 1}, h0);
                CHECK(kin.forward(theta, pose) >= 0);
                worstPos = std::max(worstPos, (pose.p - target.p).norm());
                worstAng = std::max(worstAng, rotationVector(pose.R * target.R.transposed()).norm());
                worstTorsion = std::max(worstTorsion, std::fabs(torsion(pose.R)));
                for (int i = 0; i < 3; ++i)
                    worstPlane = std::max(worstPlane,
                                          std::fabs(kin.geometry().tangential(i).dot(
                                              target.toWorld(kin.geometry().jointInPlate(i)))));
                ++n;
            }
        }
    }
    std::printf("    %d poses: max position error %.2e m, max orientation error %.2e rad\n", n, worstPos, worstAng);
    std::printf("    max torsion %.2e rad, max leg-plane residual %.2e m\n", worstTorsion, worstPlane);
    CHECK(worstPos < 1e-9);
    CHECK(worstAng < 1e-9);
    CHECK(worstTorsion < 1e-9);
    CHECK(worstPlane < 1e-12);
}

TEST(Kinematics_ParasiticMotionClosedForm) {
    // Centre of the joint circle: p_J = -(r/2)(1 - cos tilt)(cos g, sin g), g = 3 beta_0 - 2 phi.
    const Kinematics<double> kin(PlatformGeometry<double>::fromCad());
    const auto& g = kin.geometry();
    bbptest::Csv csv("kin_parasitic.csv", "tilt_deg,azimuth_deg,px_mm,py_mm,parasitic_mm,closed_form_mm,theta0_deg,theta1_deg,theta2_deg");
    double worst = 0.0;
    for (double az = 0.0; az < 360.0; az += 30.0) {
        for (double tilt = 0.0; tilt <= 15.0 + 1e-9; tilt += 0.5) {
            const V nrm{std::sin(tilt * kDeg) * std::cos(az * kDeg), std::sin(tilt * kDeg) * std::sin(az * kDeg),
                        std::cos(tilt * kDeg)};
            double theta[3];
            Pose<double> pose;
            kin.inverse(nrm, kin.levelHeight(0.0), theta, &pose);
            const V pJ = pose.p - pose.normal() * g.jointDepth;
            const double gam = 3.0 * g.legAzimuth[0] - 2.0 * az * kDeg;
            const double s = -0.5 * g.jointRadius * (1.0 - std::cos(tilt * kDeg));
            worst = std::max(worst, std::hypot(pJ.x - s * std::cos(gam), pJ.y - s * std::sin(gam)));
            csv.row(tilt, az, pose.p.x * 1e3, pose.p.y * 1e3, std::hypot(pJ.x, pJ.y) * 1e3, std::fabs(s) * 1e3,
                    theta[0] / kDeg, theta[1] / kDeg, theta[2] / kDeg);
        }
    }
    std::printf("    max deviation from closed form %.2e m\n", worst);
    bbptest::report("parasitic shift of joint centre at 10 deg [mm]",
                    0.5 * g.jointRadius * (1 - std::cos(10 * kDeg)) * 1e3, 0.5 * 94.0 * (1 - std::cos(10 * kDeg)));
    CHECK(worst < 1e-12);
}

TEST(Kinematics_JacobianMatchesFiniteDifferences) {
    const Kinematics<double> kin(PlatformGeometry<double>::fromCad());
    double theta[3] = {0.12, -0.20, 0.05};
    Pose<double> pose = kin.poseFromNormal(V{0, 0, 1}, kin.levelHeight(0.0));
    CHECK(kin.forward(theta, pose) >= 0);
    double J[6][3];
    CHECK(kin.jacobian(theta, pose, J));
    double worst = 0.0;
    for (int c = 0; c < 3; ++c) {
        const double h = 1e-6;
        double tp[3] = {theta[0], theta[1], theta[2]}, tm[3] = {theta[0], theta[1], theta[2]};
        tp[c] += h;
        tm[c] -= h;
        Pose<double> pp = pose, pm = pose;
        kin.forward(tp, pp);
        kin.forward(tm, pm);
        const V dv = (pp.p - pm.p) / (2 * h);
        const V dw = rotationVector(pp.R * pm.R.transposed()) / (2 * h);
        const double fd[6] = {dv.x, dv.y, dv.z, dw.x, dw.y, dw.z};
        for (int r = 0; r < 6; ++r) worst = std::max(worst, std::fabs(fd[r] - J[r][c]));
    }
    std::printf("    max |J - J_fd| = %.2e\n", worst);
    CHECK(worst < 1e-7);
}

TEST(Kinematics_FloatBuildAgreesWithDouble) {
    // The firmware runs the same code in single precision.
    const Kinematics<double> kd(PlatformGeometry<double>::fromCad());
    const Kinematics<float> kf(PlatformGeometry<float>::fromCad());
    double worst = 0.0;
    for (double sx = -10.0; sx <= 10.0; sx += 2.5) {
        for (double sy = -10.0; sy <= 10.0; sy += 2.5) {
            double td[3];
            float tf[3];
            CHECK(kd.inverseSlopes(sx * kDeg, sy * kDeg, kd.levelHeight(0.0), td));
            CHECK(kf.inverseSlopes(float(sx * kDeg), float(sy * kDeg), kf.levelHeight(0.0f), tf));
            for (int i = 0; i < 3; ++i) worst = std::max(worst, std::fabs(td[i] - double(tf[i])));
        }
    }
    std::printf("    max |theta_float - theta_double| = %.2e rad (%.4f deg)\n", worst, worst / kDeg);
    CHECK(worst < 1e-4);
}
