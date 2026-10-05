// Ball-on-moving-plate dynamics. The ball is a rigid sphere (radius a, mass m, moment of
// inertia I = k m a^2) in unilateral frictional contact with a rigid plate whose motion is
// prescribed (position, velocity and acceleration of the plate-top origin, angular velocity
// and acceleration). See docs/physics, section "Ball-plate dynamics", for the derivation.
#pragma once

#include <functional>

#include "bbp/core/kinematics.hpp"
#include "bbp/sim/params.hpp"

namespace bbp::sim {

using Vec3d = Vec3<double>;
using Mat3d = Mat3<double>;
using Posed = Pose<double>;

enum class ContactMode : int {
    Rolling = 0,   // no-slip contact, rolling relative to the plate
    Stuck = 1,     // no-slip and no relative rolling: held by static rolling resistance
    Slipping = 2,  // sliding contact, Coulomb friction
    Flight = 3,    // no contact (bouncing or launched)
    Lost = 4,      // fell off the plate
};

const char* toString(ContactMode m);

// Full kinematic state of the plate at one instant (world frame).
struct PlateMotion {
    Posed pose;    // origin = centre of the top face
    Vec3d v;       // velocity of the origin
    Vec3d w;       // angular velocity
    Vec3d a;       // acceleration of the origin
    Vec3d alpha;   // angular acceleration

    Vec3d velocityAt(const Vec3d& x) const { return v + w.cross(x - pose.p); }
    Vec3d accelerationAt(const Vec3d& x) const {
        const Vec3d r = x - pose.p;
        return a + alpha.cross(r) + w.cross(w.cross(r));
    }
};

struct BallState {
    Vec3d r;   // centre position (world)
    Vec3d v;   // centre velocity
    Vec3d w;   // angular velocity
    ContactMode mode = ContactMode::Rolling;
    bool spinStuck = true;  // pivoting (drilling) friction holding the relative spin at zero
    Vec3d slipDir;          // slip direction while Slipping, to detect slip reversal
    double spinSign = 0.0;  // sign of the relative spin while pivoting, to detect reversal
};

// Time derivative of the continuous ball state plus the contact wrench acting on the ball.
struct BallRates {
    Vec3d dv;      // acceleration of the centre
    Vec3d dw;      // angular acceleration
    Vec3d force;   // contact force on the ball (normal + friction), world frame
    Vec3d moment;  // contact couple on the ball (rolling + pivoting resistance)
    double normalForce = 0.0;
};

struct BallEvents {
    bool impact = false;
    bool separated = false;
    bool fellOff = false;
    double impactSpeed = 0.0;
};

class BallModel {
public:
    BallModel(const BallParams& ball, const ContactParams& contact, const PlateParams& plate,
              const EnvironmentParams& env, double plateRadius);

    BallRates rates(const BallState& s, const PlateMotion& pm) const;

    // Called after each integration step: projects the state back onto the active constraints
    // and performs mode transitions (stick/roll/slip/flight) and impacts.
    void postStep(BallState& s, const PlateMotion& pm, BallEvents* ev = nullptr) const;

    // Place the ball at rest relative to the plate at plate coordinates (x, y).
    BallState restingOn(const PlateMotion& pm, double x, double y) const;

    double mass() const { return m_; }
    double radius() const { return a_; }
    double inertia() const { return I_; }
    double inertiaFactor() const { return k_; }
    double gravity() const { return g_; }
    // Maximum pivoting couple for normal force N: (3 pi / 16) mu N a_c with the Hertz contact
    // radius a_c (Johnson, Contact Mechanics, 1985).
    double pivotingCouple(double normalForce) const;
    double hertzRadius(double normalForce) const;

    // Mechanical energy relative to the world frame (for verification).
    double energy(const BallState& s) const;

private:
    Vec3d gravityAndDrag(const Vec3d& v) const;
    Vec3d resistingCouple(const BallState& s, const PlateMotion& pm, double N, const Vec3d& wRel) const;
    void projectContact(BallState& s, const PlateMotion& pm) const;
    void projectNoSlip(BallState& s, const PlateMotion& pm) const;
    // Wrench required to keep the ball rigidly attached to the plate.
    void stuckWrench(const BallState& s, const PlateMotion& pm, Vec3d& F, Vec3d& M) const;

    BallParams ball_;
    ContactParams c_;
    EnvironmentParams env_;
    double m_, a_, I_, k_, g_, plateRadius_;
    double eStar_;     // effective Hertz modulus
    double dragK_;     // 0.5 rho Cd A
};

// Fixed-step RK4 integrator for the ball alone on a prescribed plate motion. Used by the
// verification tests; the full simulator integrates the ball together with the mechanism.
class BallIntegrator {
public:
    using MotionFn = std::function<PlateMotion(double t)>;
    BallIntegrator(const BallModel& model, MotionFn motion, double dt)
        : model_(model), motion_(std::move(motion)), dt_(dt) {}

    void step(BallState& s, double& t, BallEvents* ev = nullptr) const;

private:
    const BallModel& model_;
    MotionFn motion_;
    double dt_;
};

}  // namespace bbp::sim
