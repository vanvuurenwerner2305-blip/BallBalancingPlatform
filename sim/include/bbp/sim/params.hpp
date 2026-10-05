// Physical and numerical parameters of the simulator. Every value is tagged with its source:
//   [CAD]       measured from CAD/FullASSY.step (see core/include/bbp/core/cad_geometry.hpp)
//   [datasheet] typical manufacturer data for the assumed component
//   [derived]   computed from other parameters
//   [assumed]   engineering estimate; should be identified on the real platform
// The paper in docs/physics lists the same table with references.
#pragma once

#include <cstdint>

#include "bbp/core/cad_geometry.hpp"
#include "bbp/core/kinematics.hpp"

namespace bbp::sim {

struct EnvironmentParams {
    double gravity = 9.80665;   // [m/s^2] standard gravity
    double airDensity = 1.20;   // [kg/m^3] dry air at 20 C, sea level
    double dragCoefficient = 0.47;  // [-] smooth sphere, subcritical Reynolds number
};

struct BallParams {
    double radius = cad::kBallRadius * 1e-3;  // [m] [CAD]
    double innerRadius = 0.0;                 // [m] 0 for a solid ball, >0 for a hollow shell [assumed solid]
    double density = 7810.0;                  // [kg/m^3] AISI 52100 bearing steel [assumed material]
    double youngsModulus = 210e9;             // [Pa] steel
    double poissonRatio = 0.30;               // [-] steel

    double mass() const;                      // [derived]
    // Rolling-inertia factor k = I / (m a^2): 2/5 solid, 2/3 thin shell.
    double inertiaFactor() const;             // [derived]
    double momentOfInertia() const { return inertiaFactor() * mass() * radius * radius; }
};

struct PlateParams {
    double density = 1190.0;        // [kg/m^3] cast PMMA (acrylic) [assumed material]
    double youngsModulus = 3.2e9;   // [Pa] PMMA
    double poissonRatio = 0.37;     // [-] PMMA
    double refractiveIndex = 1.49;  // [-] PMMA at 589 nm
    double absorption = 1.0;        // [1/m] bulk absorption of clear PMMA in the visible [assumed]
    double bossRadius = 5.0e-3;     // [m] opaque joint bosses under the plate [CAD]
};

struct ContactParams {
    double staticFriction = 0.45;      // [-] steel on PMMA [assumed]
    double kineticFriction = 0.40;     // [-] steel on PMMA [assumed]
    double rollingResistance = 1.0e-3; // [-] coefficient c_rr: resisting couple = c_rr * a * N [assumed]
    double restitution = 0.50;         // [-] normal coefficient of restitution [assumed]
    double restingSpeed = 0.02;        // [m/s] impacts slower than this do not bounce
    double stickSpeed = 1.0e-4;        // [m/s] Karnopp stick window for rolling resistance
    double slipSpeed = 1.0e-4;         // [m/s] slip velocity below which sliding becomes rolling
    double spinStickRate = 1.0e-3;     // [rad/s] Karnopp window for pivoting (spin) friction
};

struct MechanismParams {
    double crankDensity = 1240.0;   // [kg/m^3] PLA, solid [assumed]
    double rodDensity = 1240.0;     // [kg/m^3] PLA, solid [assumed]
    double thetaMin = -75.0;        // [deg] mechanical end stops of the crank [assumed]
    double thetaMax = 75.0;         // [deg]
    double endStopStiffness = 200.0;  // [N m/rad]
    double endStopDamping = 2.0;      // [N m s/rad]
};

// Hobby servo, modelled on an MG996R-class metal-gear servo at 6 V.
struct ServoParams {
    double stallTorque = 1.08;      // [N m] 11 kgf cm at 6 V [datasheet]
    double noLoadSpeed = 7.48;      // [rad/s] 0.14 s / 60 deg at 6 V [datasheet]
    double mechanicalTimeConstant = 0.020;  // [s] tau_m = J omega0 / tau_stall [assumed]
    double coulombFriction = 0.02;  // [N m] gear-train friction at the output [assumed]
    double proportionalBand = 6.0;  // [deg] error that saturates the drive [assumed]
    double deadbandUs = 5.0;        // [us] pulse-width deadband [datasheet]
    double controlRate = 250.0;     // [Hz] internal controller update rate [assumed]
    double potNoiseDeg = 0.05;      // [deg] 1-sigma feedback-pot noise [assumed]
    double pulseMinUs = 500.0;      // [us] pulse at -90 deg [datasheet]
    double pulseMaxUs = 2500.0;     // [us] pulse at +90 deg [datasheet]
    double pwmPeriod = 0.020;       // [s] 50 Hz servo frame
    double pulseResolutionUs = 20000.0 / 16384.0;  // [us] ESP32-S3 LEDC, 14 bit at 50 Hz
    double centreCrankAngleDeg = 0.0;  // [deg] crank angle at the 1500 us pulse [assumed mounting]
    int direction = +1;                // +1: larger pulse raises the crank pin

    double outputInertia() const { return mechanicalTimeConstant * stallTorque / noLoadSpeed; }  // [derived]
};

enum class PixelFormat : std::uint8_t { Gray8, RGB565, RGB888 };

struct CameraParams {
    // Optical centre on the central axis at the lens element [CAD]; entrance pupil assumed there.
    double x = cad::kCameraX * 1e-3, y = cad::kCameraY * 1e-3, z = cad::kCameraLensBottomZ * 1e-3;
    double yawDeg = 0.0;            // [deg] rotation of the sensor about the optical axis [assumed]
    int width = 320;                // QVGA, the ESP32-camera working resolution [assumed]
    int height = 240;
    double pinholeHfovDeg = 90.0;   // [deg] horizontal FOV of the undistorted pinhole model [assumed]
    double k1 = -0.25, k2 = 0.05, k3 = 0.0;  // radial distortion (Brown-Conrady) [assumed]
    double p1 = 0.0, p2 = 0.0;               // tangential distortion [assumed]
    double fps = 30.0;              // [Hz] [assumed]
    double exposure = 0.006;        // [s] [assumed]
    double readoutTime = 0.015;     // [s] rolling-shutter readout of all rows [assumed]
    double latency = 0.003;         // [s] DMA + hand-over after the last row [assumed]
    PixelFormat format = PixelFormat::RGB565;
    int spatialSamples = 2;         // supersampling per pixel axis (anti-aliasing)
    int temporalSamples = 4;        // samples across the exposure (motion blur)
};

struct SensorParams {
    double fullWell = 6000.0;       // [e-] [assumed, typical 2-3 um pixel]
    double readNoise = 4.0;         // [e-] [assumed]
    double skyLevel = 0.70;         // fraction of full well for unit radiance at the reference exposure
    double referenceExposure = 0.006;  // [s]
    double gamma = 2.2;             // output transfer curve
    double vignettingPower = 2.0;   // relative illumination cos^n of the field angle [assumed]
};

struct LightingParams {
    double skyRadiance[3] = {1.0, 1.0, 1.0};    // ceiling seen through the plate (relative units)
    double skyGradient = 0.15;      // linear brightness gradient across the ceiling (window side +x)
    double baseRadiance[3] = {0.05, 0.05, 0.05};  // what the ball's underside sees (base, shadow)
    double bossRadiance[3] = {0.08, 0.08, 0.08};
    double ledIntensity = 0.0;      // optional ring light at the camera, radiant intensity [rel. units m^2]
    // Ball surface (defaults: polished steel)
    double ballAlbedo[3] = {0.20, 0.20, 0.21};
    double ballSpecular = 0.6;
    double ballShininess = 80.0;
    double ballMirror = 0.45;
};

struct NumericsParams {
    double dt = 2.5e-4;             // [s] fixed physics step (RK4)
    double historyWindow = 0.25;    // [s] state history kept for the camera renderer
    int renderThreads = 0;          // 0: use std::thread::hardware_concurrency()
    std::uint64_t seed = 0x5eed;    // seed for all stochastic processes (noise)
};

struct SimParams {
    PlatformGeometry<double> geometry = PlatformGeometry<double>::fromCad();
    EnvironmentParams env;
    BallParams ball;
    PlateParams plate;
    ContactParams contact;
    MechanismParams mechanism;
    ServoParams servo;
    CameraParams camera;
    SensorParams sensor;
    LightingParams lighting;
    NumericsParams numerics;
};

}  // namespace bbp::sim
