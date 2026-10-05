// Project settings (the Setup tab), passed to the runner as a key = value file.
#pragma once

#include <map>
#include <string>

#include "bbp/sim/params.hpp"

namespace bbp::fw {

enum class TargetMode { Fixed, Circle, Square, Figure8 };

struct EffectSettings {
    double sensorDelay = 0.0;     // [s] extra delay before a frame reaches track()
    double fpsCap = 0.0;          // [Hz] 0 = no cap
    double motorDelay = 0.0;      // [s] extra delay before a command reaches the servos
    double positionNoise = 0.0;   // [m] 1-sigma noise added to the measured position
    bool cpuModel = true;         // delay outputs by the estimated FireBeetle run time
    double cpuFactor = 40.0;      // FireBeetle time / PC time [assumed, to be calibrated]
};

struct ControlSettings {
    double maxAngleDeg = 8.0;
    double velocityFilter = 0.5;  // 0 = raw finite difference, ->1 = heavy smoothing
};

struct TargetSettings {
    TargetMode mode = TargetMode::Fixed;
    double x = 0.0, y = 0.0;      // [m] centre / fixed position
    double radius = 0.04;         // [m]
    double period = 8.0;          // [s]
};

struct Settings {
    sim::SimParams sim;
    EffectSettings effects;
    ControlSettings control;
    TargetSettings target;
    double ballStartX = 0.05, ballStartY = -0.03;  // [m]
    double speed = 1.0;            // real-time factor
    std::string ballMaterial = "steel";
};

// Parse "key = value" lines ('#' starts a comment). Unknown keys are reported in `warnings`.
Settings loadSettings(const std::string& path, std::string& warnings);

}  // namespace bbp::fw
