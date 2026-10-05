// Kinematics of the 3-RRS parallel platform (three servo cranks, each driving a rod whose
// lower end is a revolute joint on the crank pin and whose upper end is a spherical joint
// under the plate). Shared by the firmware (T = float) and the simulator (T = double).
//
// Conventions (see docs/physics, section "Kinematics"):
//  * World frame W: fixed to the base, z up, origin on the central axis at the height of the
//    plate's bottom face in the CAD pose (i.e. the CAD frame, in metres).
//  * Plate frame P: origin at the centre of the plate's TOP surface, z along the plate normal.
//  * Leg i lies in the vertical plane through the central axis at azimuth beta_i, with unit
//    vectors e_i (radially outward), t_i = e_z x e_i (tangential) and e_z.
//  * Crank angle theta_i is measured from e_i towards +e_z (pin up = positive).
//  * Plate orientation uses tilt-and-torsion angles R = Rz(phi) Ry(tilt) Rz(sigma - phi);
//    for this symmetric mechanism the torsion sigma is identically zero.
#pragma once

#include "bbp/core/cad_geometry.hpp"
#include "bbp/core/math.hpp"

namespace bbp {

template <typename T>
struct Pose {
    Mat3<T> R = Mat3<T>::identity();  // plate -> world rotation
    Vec3<T> p{};                      // world position of the plate-top centre

    Vec3<T> normal() const { return R.col(2); }
    Vec3<T> toWorld(const Vec3<T>& x_plate) const { return p + R * x_plate; }
    Vec3<T> toPlate(const Vec3<T>& x_world) const { return R.transposed() * (x_world - p); }
};

template <typename T>
struct PlatformGeometry {
    T legAzimuth[3];  // beta_i [rad]
    T pivotRadius;    // horizontal distance of the crank axis from the central axis [m]
    T pivotZ;         // world z of the crank axis [m]
    T crankLength;    // L1 [m]
    T rodLength;      // L2 [m]
    T jointRadius;    // radius of the spherical-joint circle on the plate [m]
    T jointDepth;     // distance of the spherical-joint plane below the plate top [m]
    T plateRadius;    // [m]
    T plateThickness; // [m]

    // Geometry from the CAD model, idealised to exact 120-degree symmetry with leg planes
    // through the central axis (the CAD assembly has them 0.77 mm off-axis; see the paper).
    static PlatformGeometry fromCad() {
        PlatformGeometry g{};
        for (int i = 0; i < 3; ++i) g.legAzimuth[i] = deg2rad(T(cad::kLegAzimuthDeg[i]));
        const T mm = T(1e-3);
        g.pivotRadius = T(cad::kPivotRadius) * mm;
        g.pivotZ = T(cad::kPivotZ) * mm;
        g.crankLength = T(cad::kCrankLength) * mm;
        g.rodLength = T(cad::kRodLength) * mm;
        g.jointRadius = T(cad::kJointRadius) * mm;
        g.jointDepth = T(cad::kPlateTopZ - cad::kJointZ) * mm;
        g.plateRadius = T(cad::kPlateRadius) * mm;
        g.plateThickness = T(cad::kPlateTopZ - cad::kPlateBottomZ) * mm;
        return g;
    }

    Vec3<T> radial(int i) const { return {std::cos(legAzimuth[i]), std::sin(legAzimuth[i]), T(0)}; }
    Vec3<T> tangential(int i) const { return {-std::sin(legAzimuth[i]), std::cos(legAzimuth[i]), T(0)}; }
    Vec3<T> pivot(int i) const { return radial(i) * pivotRadius + Vec3<T>{T(0), T(0), pivotZ}; }
    Vec3<T> crankPin(int i, T theta) const {
        return pivot(i) + (radial(i) * std::cos(theta) + Vec3<T>{T(0), T(0), std::sin(theta)}) * crankLength;
    }
    // d(crankPin)/d(theta)
    Vec3<T> crankPinRate(int i, T theta) const {
        return (radial(i) * -std::sin(theta) + Vec3<T>{T(0), T(0), std::cos(theta)}) * crankLength;
    }
    // Spherical joint centre in plate coordinates.
    Vec3<T> jointInPlate(int i) const {
        return {jointRadius * std::cos(legAzimuth[i]), jointRadius * std::sin(legAzimuth[i]), -jointDepth};
    }
};

template <typename T>
class Kinematics {
public:
    explicit Kinematics(const PlatformGeometry<T>& g) : g_(g) {}

    const PlatformGeometry<T>& geometry() const { return g_; }

    // Plate unit normal for given surface slope angles: slopeX is the angle the plate surface
    // makes with the horizontal along the world x direction (surface rises towards +x when
    // positive), likewise slopeY. Unlike Euler angles this does not depend on a rotation order.
    static Vec3<T> normalFromSlopes(T slopeX, T slopeY) {
        return Vec3<T>{-std::tan(slopeX), -std::tan(slopeY), T(1)}.normalized();
    }

    // Plate pose with the given normal and plate-top height, including the parasitic
    // translation and (zero) torsion that the leg-plane constraints impose.
    Pose<T> poseFromNormal(const Vec3<T>& normal, T height) const {
        const Vec3<T> n = normal.normalized();
        T cz = n.z;
        if (cz > T(1)) cz = T(1);
        const T tilt = std::acos(cz);
        const T phi = std::atan2(n.y, n.x);
        Pose<T> pose;
        pose.R = rotationZ(phi) * rotationY(tilt) * rotationZ(-phi);
        // Closed form for 120-degree symmetric legs (derived in the paper): the centre of the
        // joint circle moves by -(r/2)(1 - cos tilt)(cos g, sin g), g = 3 beta_0 - 2 phi.
        const T gam = T(3) * g_.legAzimuth[0] - T(2) * phi;
        const T s = -T(0.5) * g_.jointRadius * (T(1) - std::cos(tilt));
        const Vec3<T> nW = pose.R.col(2);
        pose.p = Vec3<T>{s * std::cos(gam), s * std::sin(gam), T(0)} + nW * g_.jointDepth;
        pose.p.z = height;
        // Newton polish on (px, py, torsion) so the result is exact for any leg layout and
        // to machine precision for the symmetric one.
        for (int it = 0; it < 3; ++it) {
            T A[3][3], b[3];
            T resid = T(0);
            for (int i = 0; i < 3; ++i) {
                const Vec3<T> t = g_.tangential(i);
                const Vec3<T> Rb = pose.R * g_.jointInPlate(i);
                b[i] = -t.dot(pose.p + Rb);
                resid += std::fabs(b[i]);
                A[i][0] = t.x;
                A[i][1] = t.y;
                A[i][2] = t.dot(nW.cross(Rb));
            }
            if (resid < T(1e-7) * g_.jointRadius) break;
            if (!solveLinear(A, b)) break;
            pose.p.x += b[0];
            pose.p.y += b[1];
            pose.R = rotationFromRotationVector(nW * b[2]) * pose.R;
        }
        return pose;
    }

    // Crank angle that places leg i's spherical joint at world point B. Returns false if the
    // point is out of reach of the crank/rod pair.
    bool crankAngleFor(int i, const Vec3<T>& B, T& theta) const {
        const T u = g_.radial(i).dot(B) - g_.pivotRadius;
        const T v = B.z - g_.pivotZ;
        const T rho = std::sqrt(u * u + v * v);
        const T L1 = g_.crankLength, L2 = g_.rodLength;
        const T K = (u * u + v * v + L1 * L1 - L2 * L2) / (T(2) * L1);
        if (rho < T(1e-9) || std::fabs(K) > rho) return false;
        // Branch with the crank pin outside/below the pivot-joint line (the assembled one).
        theta = std::atan2(v, u) - std::acos(K / rho);
        return true;
    }

    bool inverse(const Pose<T>& pose, T (&theta)[3]) const {
        for (int i = 0; i < 3; ++i) {
            if (!crankAngleFor(i, pose.toWorld(g_.jointInPlate(i)), theta[i])) return false;
        }
        return true;
    }

    // Inverse kinematics from plate normal and plate-top height.
    bool inverse(const Vec3<T>& normal, T height, T (&theta)[3], Pose<T>* poseOut = nullptr) const {
        const Pose<T> pose = poseFromNormal(normal, height);
        if (poseOut) *poseOut = pose;
        return inverse(pose, theta);
    }

    bool inverseSlopes(T slopeX, T slopeY, T height, T (&theta)[3]) const {
        return inverse(normalFromSlopes(slopeX, slopeY), height, theta);
    }

    // Plate-top height when the plate is level and every crank is at angle theta.
    T levelHeight(T theta) const {
        const Vec3<T> P = g_.crankPin(0, theta);
        const T du = g_.jointRadius - g_.radial(0).dot(P);
        return P.z + std::sqrt(g_.rodLength * g_.rodLength - du * du) + g_.jointDepth;
    }

    // Forward kinematics by Newton iteration on the six loop-closure constraints, starting
    // from (and overwriting) `pose`. Returns the iteration count, or -1 on failure.
    int forward(const T (&theta)[3], Pose<T>& pose, int maxIter = 30, T tol = T(1e-10)) const {
        for (int it = 0; it < maxIter; ++it) {
            T A[6][6], b[6];
            T resid = T(0);
            buildConstraintJacobian(theta, pose, A, b);
            for (int k = 0; k < 6; ++k) resid += std::fabs(b[k]);
            if (resid < tol) return it;
            for (int k = 0; k < 6; ++k) b[k] = -b[k];
            if (!solveLinear(A, b)) return -1;
            pose.p += Vec3<T>{b[0], b[1], b[2]};
            pose.R = orthonormalized(rotationFromRotationVector(Vec3<T>{b[3], b[4], b[5]}) * pose.R);
        }
        return -1;
    }

    // Velocity Jacobian: [v_p; omega] = J * thetaDot, where v_p is the velocity of the plate-top
    // centre and omega the plate angular velocity (world frame). Returns false at a
    // singularity of the constraint Jacobian.
    bool jacobian(const T (&theta)[3], const Pose<T>& pose, T (&J)[6][3]) const {
        T A[6][6], g[6];
        buildConstraintJacobian(theta, pose, A, g);
        for (int i = 0; i < 3; ++i) {
            T Ac[6][6], col[6] = {};
            for (int r = 0; r < 6; ++r)
                for (int c = 0; c < 6; ++c) Ac[r][c] = A[r][c];
            // Phi_x xi + Phi_theta thetaDot = 0 ; only rod row i depends on theta_i.
            const Vec3<T> d = (pose.toWorld(g_.jointInPlate(i)) - g_.crankPin(i, theta[i])) / g_.rodLength;
            col[2 * i] = d.dot(g_.crankPinRate(i, theta[i]));
            if (!solveLinear(Ac, col)) return false;
            for (int r = 0; r < 6; ++r) J[r][i] = col[r];
        }
        return true;
    }

private:
    // Rows 2i: rod-length constraint (|B_i - P_i|^2 - L2^2) / (2 L2)   [m]
    // Rows 2i+1: leg-plane constraint t_i . B_i                         [m]
    // Columns: variation of the plate pose as a twist (dp, dphi) in the world frame.
    void buildConstraintJacobian(const T (&theta)[3], const Pose<T>& pose, T (&A)[6][6], T (&g)[6]) const {
        for (int i = 0; i < 3; ++i) {
            const Vec3<T> Rb = pose.R * g_.jointInPlate(i);
            const Vec3<T> B = pose.p + Rb;
            const Vec3<T> d = B - g_.crankPin(i, theta[i]);
            const Vec3<T> t = g_.tangential(i);
            g[2 * i] = (d.squaredNorm() - g_.rodLength * g_.rodLength) / (T(2) * g_.rodLength);
            g[2 * i + 1] = t.dot(B);
            const Vec3<T> dn = d / g_.rodLength;
            const Vec3<T> r1 = Rb.cross(dn);
            const Vec3<T> r2 = Rb.cross(t);
            const T row1[6] = {dn.x, dn.y, dn.z, r1.x, r1.y, r1.z};
            const T row2[6] = {t.x, t.y, t.z, r2.x, r2.y, r2.z};
            for (int c = 0; c < 6; ++c) {
                A[2 * i][c] = row1[c];
                A[2 * i + 1][c] = row2[c];
            }
        }
    }

    PlatformGeometry<T> g_;
};

}  // namespace bbp
