#include "settings.hpp"

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
        b.density = 2500.0;
        b.innerRadius = 0.0;
        b.youngsModulus = 70e9;
        b.poissonRatio = 0.22;
        albedo(0.35, 0.45, 0.40);
        L.ballMirror = 0.15;
        L.ballSpecular = 0.8;
    } else if (m == "plastic_white") {
        b.density = 1200.0;
        b.innerRadius = 0.0;
        b.youngsModulus = 2.5e9;
        b.poissonRatio = 0.38;
        albedo(0.85, 0.85, 0.82);
        L.ballMirror = 0.03;
        L.ballSpecular = 0.2;
    } else if (m == "hollow_orange") {
        b.density = 1050.0;
        b.innerRadius = 0.0094;  // 0.6 mm wall
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

    std::string material = "steel";
    str("ball.material", material);
    applyMaterial(s, material);
    num("ball.start_x_mm", s.ballStartX, 1e-3);
    num("ball.start_y_mm", s.ballStartY, 1e-3);

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

    num("physics.rolling_resistance", s.sim.contact.rollingResistance);
    double mu = s.sim.contact.staticFriction;
    num("physics.friction", mu);
    s.sim.contact.staticFriction = mu;
    s.sim.contact.kineticFriction = 0.9 * mu;

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
