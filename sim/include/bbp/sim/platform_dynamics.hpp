// Rigid-body dynamics of the 3-RRS mechanism in the three crank angles (reduced
// coordinates), derived with the principle of virtual work: the plate's Newton-Euler
// wrench is mapped to the cranks through the transposed kinematic Jacobian.
#pragma once

#include "bbp/sim/ball.hpp"
#include "bbp/sim/params.hpp"

namespace bbp::sim {

struct InertialProperties {
    double mass = 0.0;
    Vec3d com;        // plate frame
    Mat3d inertia;    // about the centre of mass, plate frame
};

class PlatformDynamics {
public:
    explicit PlatformDynamics(const SimParams& p);

    const Kinematics<double>& kinematics() const { return kin_; }
    const InertialProperties& plateInertia() const { return plate_; }
    double crankInertia() const { return crankInertia_; }   // per crank, about its axis, incl. servo
    double crankMass() const { return crankMass_; }
    double rodMass() const { return rodMass_; }

    // Crank accelerations for given crank angles/rates, actuator torques and an external wrench
    // on the plate (force applied at `forcePoint` plus a free couple), and the resulting plate
    // motion. `pose` is the warm start for the forward kinematics and is updated.
    bool evaluate(const double (&theta)[3], const double (&thetaDot)[3], const double (&tau)[3],
                  const Vec3d& force, const Vec3d& forcePoint, const Vec3d& couple, Posed& pose,
                  double (&thetaDDot)[3], PlateMotion& motion) const;

    // Gravity torque on crank i (crank's own mass plus the half rod mass at its pin).
    double crankGravityTorque(double theta) const;
    // End-stop torque (penalty spring-damper outside the mechanical range).
    double endStopTorque(double theta, double thetaDot) const;

private:
    Kinematics<double> kin_;
    InertialProperties plate_;   // plate plus the half rod masses lumped at the spherical joints
    double crankInertia_ = 0.0;
    double crankMass_ = 0.0, rodMass_ = 0.0;
    double crankComDist_ = 0.0, crankComAngle_ = 0.0;
    double g_;
    MechanismParams mech_;
};

}  // namespace bbp::sim
