// End-to-end demonstration of the simulator: a reference vision pipeline and PD controller,
// of the kind a student would write, close the loop through the simulated camera and servos.
//
//   bbp_sim_demo [output_dir]
//
// Writes closed_loop.csv (ground truth vs. measurement) and a few camera frames (PPM).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "bbp/sim/world.hpp"

using namespace bbp;
using namespace bbp::sim;

namespace {
const double kDeg = 3.14159265358979323846 / 180.0;

void writePpm(const std::string& path, const Frame& f) {
    std::ofstream o(path, std::ios::binary);
    o << "P6\n" << f.width << " " << f.height << "\n255\n";
    for (int y = 0; y < f.height; ++y)
        for (int x = 0; x < f.width; ++x) {
            std::uint8_t r, g, b;
            f.rgb(x, y, r, g, b);
            o.put(char(r)).put(char(g)).put(char(b));
        }
}

// Dark-blob tracker: threshold on luma inside the plate, ignoring the joint bosses, and take
// the intensity-weighted centroid in a window around the previous detection.
struct Tracker {
    bool have = false;
    double u = 0, v = 0;
    std::vector<std::pair<double, double>> bossPx;

    bool update(const Frame& f) {
        auto luma = [&](int x, int y) {
            std::uint8_t r, g, b;
            f.rgb(x, y, r, g, b);
            return 0.299 * r + 0.587 * g + 0.114 * b;
        };
        const int win = have ? 25 : 1000;
        const int x0 = std::max(0, int(u) - win), x1 = std::min(f.width, int(u) + win);
        const int y0 = std::max(0, int(v) - win), y1 = std::min(f.height, int(v) + win);
        const double thr = 110.0;
        double sw = 0, su = 0, sv = 0;
        for (int y = y0; y < y1; ++y)
            for (int x = x0; x < x1; ++x) {
                bool nearBoss = false;
                for (auto& b : bossPx) nearBoss |= std::hypot(x - b.first, y - b.second) < 14.0;
                if (nearBoss) continue;
                const double l = luma(x, y);
                if (l < thr) {
                    sw += thr - l;
                    su += (thr - l) * (x + 0.5);
                    sv += (thr - l) * (y + 0.5);
                }
            }
        if (sw < 50.0) {
            have = false;
            return false;
        }
        u = su / sw;
        v = sv / sw;
        have = true;
        return true;
    }
};
}  // namespace

int main(int argc, char** argv) {
    const std::string out = argc > 1 ? argv[1] : ".";
    SimParams p;
    SimWorld world(p);
    world.reset(0.05, -0.03);
    const auto& kin = world.kinematics();
    const auto& cam = world.camera();
    const double h0 = world.neutralHeight();

    Tracker tracker;
    {
        const Pose<double> level = kin.poseFromNormal(Vec3d{0, 0, 1}, h0);
        for (int i = 0; i < 3; ++i) {
            double u, v;
            cam.projectDirect(level.toWorld(p.geometry.jointInPlate(i)), u, v);
            tracker.bossPx.push_back({u, v});
        }
    }

    // PD controller on the measured position (finite-difference velocity, first-order filter).
    const double g = p.env.gravity, k = p.ball.inertiaFactor();
    const double wn = 2.0 * 3.14159265358979323846 * 0.55, zeta = 0.85;
    double lastX = 0, lastY = 0, lastT = -1, vx = 0, vy = 0;
    double slopeX = 0, slopeY = 0;
    Pose<double> commandedPose = kin.poseFromNormal(Vec3d{0, 0, 1}, h0);

    std::ofstream csv(out + "/closed_loop.csv");
    csv << "t,true_x,true_y,meas_x,meas_y,target_x,target_y,slope_cmd_x,slope_cmd_y,slope_x,slope_y,"
           "theta0,theta1,theta2,mode,frame_age\n";
    const auto wall0 = std::chrono::steady_clock::now();
    const double duration = 12.0;
    int framesSaved = 0;
    Frame f;
    while (world.time() < duration) {
        world.advance(0.001);
        const double t = world.time();
        const double tx = t < 4.0 ? 0.0 : (t < 8.0 ? 0.04 : -0.03);
        const double ty = t < 4.0 ? 0.0 : (t < 8.0 ? 0.0 : 0.03);
        while (world.pollFrame(f)) {
            const GroundTruth gt = world.truth();
            if (framesSaved < 3 && (f.index == 15 || f.index == 135 || f.index == 260)) {
                writePpm(out + "/frame_" + std::to_string(f.index) + ".ppm", f);
                ++framesSaved;
            }
            double mx = NAN, my = NAN;
            if (tracker.update(f)) {
                Vec3d xp;
                if (cam.backprojectToPlate(tracker.u, tracker.v, commandedPose, p.geometry.plateRadius,
                                           p.ball.radius, xp)) {
                    mx = xp.x;
                    my = xp.y;
                    if (lastT >= 0) {
                        const double dt = t - lastT, a = 0.5;
                        vx = a * vx + (1 - a) * (mx - lastX) / dt;
                        vy = a * vy + (1 - a) * (my - lastY) / dt;
                    }
                    lastX = mx;
                    lastY = my;
                    lastT = t;
                    const double ax = wn * wn * (tx - mx) - 2 * zeta * wn * vx;
                    const double ay = wn * wn * (ty - my) - 2 * zeta * wn * vy;
                    slopeX = std::asin(std::clamp(-ax * (1 + k) / g, -0.14, 0.14));
                    slopeY = std::asin(std::clamp(-ay * (1 + k) / g, -0.14, 0.14));
                    double th[3];
                    if (kin.inverse(Kinematics<double>::normalFromSlopes(slopeX, slopeY), h0, th, &commandedPose))
                        for (int i = 0; i < 3; ++i) world.setServoPulse(i, world.pulseForCrankAngle(i, th[i]));
                }
            }
            csv << t << "," << gt.ballPlate.x << "," << gt.ballPlate.y << "," << mx << "," << my << "," << tx << ","
                << ty << "," << slopeX << "," << slopeY << "," << gt.slopeX << "," << gt.slopeY << ","
                << gt.theta[0] << "," << gt.theta[1] << "," << gt.theta[2] << "," << int(gt.mode) << ","
                << t - f.exposureStart << "\n";
        }
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    const GroundTruth gt = world.truth();
    std::printf("simulated %.1f s in %.2f s wall time (%.1fx real time)\n", duration, wall, duration / wall);
    std::printf("final ball position (%.2f, %.2f) mm, target (-30, 30) mm, mode %s\n", gt.ballPlate.x * 1e3,
                gt.ballPlate.y * 1e3, toString(gt.mode));
    return 0;
}
