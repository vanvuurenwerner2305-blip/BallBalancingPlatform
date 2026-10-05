#include "bbp/sim/platform_dynamics.hpp"

#include <cmath>

namespace bbp::sim {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kMm = 1e-3;
constexpr double kMm5 = 1e-15;  // mm^5 -> m^5

Mat3d outer(const Vec3d& a, const Vec3d& b) {
    Mat3d r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a[i] * b[j];
    return r;
}

// Inertia of a point mass m at offset d from the reference point.
Mat3d pointInertia(double m, const Vec3d& d) {
    return (Mat3d::identity() * d.squaredNorm() - outer(d, d)) * m;
}
}  // namespace

PlatformDynamics::PlatformDynamics(const SimParams& p)
    : kin_(p.geometry), g_(p.env.gravity), mech_(p.mechanism) {
    const auto& geo = p.geometry;
    // Plate body from CAD: mass, centre of mass and inertia (scaled by the assumed density).
    const double mPlate = cad::kPlateVolume * 1e-9 * p.plate.density;
    const Vec3d cPlate{0.0, 0.0, (cad::kPlateComZ - cad::kPlateTopZ) * kMm};
    const Mat3d IPlate = Mat3d::diagonal(cad::kPlateIxxPerDensity, cad::kPlateIyyPerDensity,
                                         cad::kPlateIzzPerDensity) * (kMm5 * p.plate.density);
    rodMass_ = cad::kRodVolume * 1e-9 * p.mechanism.rodDensity;
    crankMass_ = cad::kCrankVolume * 1e-9 * p.mechanism.crankDensity;

    // Two-point lumping of each rod: half its mass at the spherical joint (moves with the plate)
    // and half at the crank pin (moves with the crank).
    double M = mPlate;
    Vec3d mc = cPlate * mPlate;
    for (int i = 0; i < 3; ++i) {
        M += 0.5 * rodMass_;
        mc += geo.jointInPlate(i) * (0.5 * rodMass_);
    }
    const Vec3d c = mc / M;
    Mat3d I = IPlate + pointInertia(mPlate, cPlate - c);
    for (int i = 0; i < 3; ++i) I = I + pointInertia(0.5 * rodMass_, geo.jointInPlate(i) - c);
    plate_.mass = M;
    plate_.com = c;
    plate_.inertia = I;

    // Crank: inertia about its axis from CAD, plus half the rod at the pin, plus the servo's
    // reflected rotor inertia.
    const double Icrank = cad::kCrankIPivotPerDensity * kMm5 * p.mechanism.crankDensity;
    crankInertia_ = Icrank + 0.5 * rodMass_ * geo.crankLength * geo.crankLength + p.servo.outputInertia();
    // Crank centre of mass: CAD gives it relative to the pivot in the CAD pose.
    crankComDist_ = std::hypot(cad::kCrankComU, cad::kCrankComV) * kMm;
    crankComAngle_ = std::atan2(cad::kCrankComV, cad::kCrankComU) - cad::kCrankAngleInCadDeg * kDeg;
}

double PlatformDynamics::crankGravityTorque(double theta) const {
    // V = m g h  ->  tau = -dV/dtheta
    const double L1 = kin_.geometry().crankLength;
    return -crankMass_ * g_ * crankComDist_ * std::cos(theta + crankComAngle_) -
           0.5 * rodMass_ * g_ * L1 * std::cos(theta);
}

double PlatformDynamics::endStopTorque(double theta, double thetaDot) const {
    const double lo = mech_.thetaMin * kDeg, hi = mech_.thetaMax * kDeg;
    if (theta > hi) return -mech_.endStopStiffness * (theta - hi) - mech_.endStopDamping * thetaDot;
    if (theta < lo) return -mech_.endStopStiffness * (theta - lo) - mech_.endStopDamping * thetaDot;
    return 0.0;
}

bool PlatformDynamics::evaluate(const double (&th)[3], const double (&thd)[3], const double (&tau)[3],
                                const Vec3d& force, const Vec3d& forcePoint, const Vec3d& couple,
                                Posed& pose, double (&thdd)[3], PlateMotion& pm) const {
    if (kin_.forward(th, pose) < 0) return false;
    double J[6][3];
    if (!kin_.jacobian(th, pose, J)) return false;

    // Velocity-product term kappa = Jdot * thetaDot by a central difference of J along thetaDot.
    double kappa[6] = {0, 0, 0, 0, 0, 0};
    const double speed = std::sqrt(thd[0] * thd[0] + thd[1] * thd[1] + thd[2] * thd[2]);
    if (speed > 1e-12) {
        const double eps = 1e-6;
        double thP[3], thM[3], JP[6][3], JM[6][3];
        for (int i = 0; i < 3; ++i) {
            thP[i] = th[i] + eps * thd[i] / speed;
            thM[i] = th[i] - eps * thd[i] / speed;
        }
        Posed pP = pose, pM = pose;
        if (kin_.forward(thP, pP) < 0 || kin_.forward(thM, pM) < 0) return false;
        if (!kin_.jacobian(thP, pP, JP) || !kin_.jacobian(thM, pM, JM)) return false;
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 3; ++c) kappa[r] += (JP[r][c] - JM[r][c]) / (2.0 * eps) * speed * thd[c];
    }

    Vec3d v{0, 0, 0}, w{0, 0, 0};
    for (int c = 0; c < 3; ++c) {
        v += Vec3d{J[0][c], J[1][c], J[2][c]} * thd[c];
        w += Vec3d{J[3][c], J[4][c], J[5][c]} * thd[c];
    }

    // Spatial inertia of the plate about its origin p (world frame).
    const Vec3d r = pose.R * plate_.com;
    const Mat3d IW = pose.R * plate_.inertia * pose.R.transposed();
    const double m = plate_.mass;
    const Mat3d S = skew(r);
    // H = [[m I, -m S], [m S, IW - m S S]]
    double H[6][6];
    const Mat3d Hvv = Mat3d::identity() * m, Hvw = S * -m, Hwv = S * m, Hww = IW - S * S * m;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            H[i][j] = Hvv.m[i][j];
            H[i][j + 3] = Hvw.m[i][j];
            H[i + 3][j] = Hwv.m[i][j];
            H[i + 3][j + 3] = Hww.m[i][j];
        }
    // Velocity-product wrench and external wrench, both about p.
    const Vec3d fv = w.cross(w.cross(r)) * m;
    const Vec3d mv = r.cross(fv) + w.cross(IW * w);
    const Vec3d fg{0.0, 0.0, -m * g_};
    const Vec3d fExt = fg + force;
    const Vec3d mExt = r.cross(fg) + (forcePoint - pose.p).cross(force) + couple;
    double wrench[6] = {fExt.x - fv.x, fExt.y - fv.y, fExt.z - fv.z, mExt.x - mv.x, mExt.y - mv.y, mExt.z - mv.z};
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) wrench[i] -= H[i][j] * kappa[j];

    // Reduced equations: (D + J^T H J) thetaDDot = tau + tau_g + J^T (W_ext - H kappa - W_v)
    double HJ[6][3] = {};
    for (int i = 0; i < 6; ++i)
        for (int c = 0; c < 3; ++c)
            for (int k = 0; k < 6; ++k) HJ[i][c] += H[i][k] * J[k][c];
    double A[3][3], b[3];
    for (int a = 0; a < 3; ++a) {
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 6; ++k) s += J[k][a] * HJ[k][c];
            A[a][c] = s + (a == c ? crankInertia_ : 0.0);
        }
        double s = 0.0;
        for (int k = 0; k < 6; ++k) s += J[k][a] * wrench[k];
        b[a] = tau[a] + crankGravityTorque(th[a]) + endStopTorque(th[a], thd[a]) + s;
    }
    if (!solveLinear(A, b)) return false;
    for (int i = 0; i < 3; ++i) thdd[i] = b[i];

    double acc[6];
    for (int rr = 0; rr < 6; ++rr) acc[rr] = kappa[rr] + J[rr][0] * thdd[0] + J[rr][1] * thdd[1] + J[rr][2] * thdd[2];
    pm.pose = pose;
    pm.v = v;
    pm.w = w;
    pm.a = Vec3d{acc[0], acc[1], acc[2]};
    pm.alpha = Vec3d{acc[3], acc[4], acc[5]};
    return true;
}

}  // namespace bbp::sim
