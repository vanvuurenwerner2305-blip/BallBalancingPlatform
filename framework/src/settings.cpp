#include "settings.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>

namespace bbp::fw {

namespace {
std::string trim(const std::string& s) {
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    const auto b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

// Surface material: appearance and elastic constants. Size and mass are set separately.
void applyMaterial(Settings& s, const std::string& m) {
    auto& b = s.sim.ball;
    auto& L = s.sim.lighting;
    s.ballMaterial = m;
    auto albedo = [&](double r, double g, double bl) {
        L.ballAlbedo[0] = r;
        L.ballAlbedo[1] = g;
        L.ballAlbedo[2] = bl;
    };
    if (m == "glass") {
        b.youngsModulus = 70e9;
        b.poissonRatio = 0.22;
        albedo(0.35, 0.45, 0.40);
        L.ballMirror = 0.15;
        L.ballSpecular = 0.8;
    } else if (m == "plastic_white") {
        b.youngsModulus = 2.5e9;
        b.poissonRatio = 0.38;
        albedo(0.85, 0.85, 0.82);
        L.ballMirror = 0.03;
        L.ballSpecular = 0.2;
    } else if (m == "hollow_orange") {
        b.youngsModulus = 2.5e9;
        b.poissonRatio = 0.38;
        albedo(0.90, 0.40, 0.08);
        L.ballMirror = 0.03;
        L.ballSpecular = 0.2;
    } else {  // steel (default)
        s.ballMaterial = "steel";
    }
}

TargetMode parseMode(const std::string& v) {
    if (v == "circle") return TargetMode::Circle;
    if (v == "square") return TargetMode::Square;
    if (v == "figure8") return TargetMode::Figure8;
    return TargetMode::Fixed;
}
}  // namespace

Settings loadSettings(const std::string& path, std::string& warnings) {
    Settings s;
    std::map<std::string, std::string> kv;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        const auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        kv[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
    }
    auto num = [&](const char* key, double& dst, double scale = 1.0) {
        auto it = kv.find(key);
        if (it == kv.end()) return;
        dst = std::atof(it->second.c_str()) * scale;
        kv.erase(it);
    };
    auto str = [&](const char* key, std::string& dst) {
        auto it = kv.find(key);
        if (it == kv.end()) return false;
        dst = it->second;
        kv.erase(it);
        return true;
    };

    // ---- ball (simulated hardware)
    std::string preset;
    str("ball.preset", preset);  // UI only: the preset's values arrive as the keys below
    std::string material = "steel";
    str("ball.material", material);
    applyMaterial(s, material);
    auto& ball = s.sim.ball;
    num("ball.radius_mm", ball.radius, 1e-3);
    double massG = ball.mass() * 1e3, wallMm = 0.0;
    num("ball.mass_g", massG);
    num("ball.wall_mm", wallMm);
    ball.radius = std::max(ball.radius, 1e-3);
    ball.innerRadius = std::clamp(ball.radius - wallMm * 1e-3, 0.0, ball.radius * 0.999);
    if (wallMm <= 0.0) ball.innerRadius = 0.0;
    const double shellVolume = 4.0 / 3.0 * 3.14159265358979323846 *
                               (std::pow(ball.radius, 3) - std::pow(ball.innerRadius, 3));
    ball.density = std::max(massG, 1e-3) * 1e-3 / shellVolume;
    num("ball.start_x_mm", s.ballStartX, 1e-3);
    num("ball.start_y_mm", s.ballStartY, 1e-3);

    // ---- contact
    auto& c = s.sim.contact;
    num("contact.static_friction", c.staticFriction);
    num("contact.kinetic_friction", c.kineticFriction);
    c.kineticFriction = std::min(c.kineticFriction, c.staticFriction);
    num("contact.rolling_resistance", c.rollingResistance);
    num("contact.restitution", c.restitution);

    // ---- servos
    auto& sv = s.sim.servo;
    num("servo.stall_torque", sv.stallTorque);
    double s60 = 60.0 * 3.14159265358979323846 / 180.0 / sv.noLoadSpeed;
    num("servo.speed_s_per_60", s60);
    sv.noLoadSpeed = 60.0 * 3.14159265358979323846 / 180.0 / std::max(s60, 1e-3);
    num("servo.time_constant_ms", sv.mechanicalTimeConstant, 1e-3);
    num("servo.deadband_us", sv.deadbandUs);
    num("servo.prop_band_deg", sv.proportionalBand);
    num("servo.control_rate_hz", sv.controlRate);
    num("servo.pot_noise_deg", sv.potNoiseDeg);

    // ---- plate
    double plateG = cad::kPlateVolume * 1e-9 * s.sim.plate.density * 1e3;
    num("plate.mass_g", plateG);
    s.sim.plate.density = plateG * 1e-3 / (cad::kPlateVolume * 1e-9);
    num("plate.refractive_index", s.sim.plate.refractiveIndex);

    // ---- experiment (also on hardware)
    num("effects.sensor_delay_ms", s.effects.sensorDelay, 1e-3);
    num("effects.fps_cap", s.effects.fpsCap);
    num("effects.motor_delay_ms", s.effects.motorDelay, 1e-3);
    num("effects.position_noise_mm", s.effects.positionNoise, 1e-3);
    double cpu = s.effects.cpuModel ? 1 : 0;
    num("effects.cpu_model", cpu);
    s.effects.cpuModel = cpu != 0;
    num("effects.cpu_factor", s.effects.cpuFactor);

    num("camera.fps", s.sim.camera.fps);
    double expMs = s.sim.camera.exposure * 1e3;
    num("camera.exposure_ms", expMs);
    s.sim.camera.exposure = expMs * 1e-3;

    // ---- camera model
    double noise = 1.0;
    num("camera.noise", noise);
    if (noise > 1e-3) {
        s.sim.sensor.fullWell /= noise * noise;  // shot noise scales with `noise`
        s.sim.sensor.readNoise /= noise;         // read noise relative to full well too
    } else {
        s.sim.sensor.fullWell = 1e12;
        s.sim.sensor.readNoise = 0.0;
    }
    double ring = 0;
    num("camera.ring_light", ring);
    s.sim.lighting.ledIntensity = ring != 0 ? 0.01 : 0.0;
    num("camera.hfov_deg", s.sim.camera.pinholeHfovDeg);
    num("camera.k1", s.sim.camera.k1);

    // ---- environment
    num("env.gravity", s.sim.env.gravity);
    double drag = 1;
    num("env.air_drag", drag);
    if (drag == 0) s.sim.env.airDensity = 0.0;

    num("control.max_angle_deg", s.control.maxAngleDeg);
    num("control.velocity_filter", s.control.velocityFilter);

    std::string mode;
    if (str("target.mode", mode)) s.target.mode = parseMode(mode);
    num("target.x_mm", s.target.x, 1e-3);
    num("target.y_mm", s.target.y, 1e-3);
    num("target.radius_mm", s.target.radius, 1e-3);
    num("target.period_s", s.target.period);

    num("sim.speed", s.speed);
    double seed = double(s.sim.numerics.seed);
    num("sim.seed", seed);
    s.sim.numerics.seed = std::uint64_t(seed);

    for (const auto& [k, v] : kv) warnings += "unknown setting '" + k + "'\n";
    return s;
}

}  // namespace bbp::fw
