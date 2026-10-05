// Simulation runner: the student's track()/control() linked with the framework and the
// simulator into one executable. The app starts it, sends commands on stdin (one per line)
// and receives a binary message stream on stdout. Anything the student prints goes to stderr.
//
//   runner <settings-file>
//
// Message framing (little endian): uint32 type, uint32 payload length, payload.
//   1 STATE  float64[kStateSize] (see below)
//   2 FRAME  uint32 index, width, height, format; float64 t; uint8 found; float32 x, y; pixels
//   3 LOG    float64 t, float64 value, utf-8 name
//   4 TEXT   uint32 level (0 info, 1 warning, 2 error), utf-8 text
// Commands: pause | resume | speed <f> | target <x_mm> <y_mm> | nudge <vx_mm_s> <vy_mm_s> | quit
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "runtime.hpp"
#include "settings.hpp"

using namespace bbp;
using namespace bbp::fw;

namespace {

constexpr int kStateSize = 48;
// STATE layout (indices into the float64 array)
enum : int {
    S_T = 0, S_RTF = 1,
    S_PLATE_P = 2,          // 3: plate-top centre [mm]
    S_PLATE_Q = 5,          // 4: quaternion w, x, y, z
    S_THETA = 9,            // 3: crank angles [rad]
    S_PIN = 12,             // 9: crank pins P_i [mm]
    S_JOINT = 21,           // 9: spherical joints B_i [mm]
    S_BALL = 30,            // 3: ball centre [mm]
    S_BALL_PLATE = 33,      // 2: true ball position on the plate [mm]
    S_MEAS_FOUND = 35,
    S_MEAS = 36,            // 2: measured position [mm]
    S_TARGET = 38,          // 2: target [mm]
    S_CMD = 40,             // 2: commanded angles [deg]
    S_SLOPE = 42,           // 2: actual plate slopes [deg]
    S_MODE = 44,
    S_CPU_MS = 45,
    S_PAUSED = 46,
    S_HOST_MS = 47,
};

FILE* gProto = nullptr;
std::mutex gProtoMutex;

void send(std::uint32_t type, const void* payload, std::uint32_t len) {
    std::lock_guard<std::mutex> lock(gProtoMutex);
    std::fwrite(&type, 4, 1, gProto);
    std::fwrite(&len, 4, 1, gProto);
    if (len) std::fwrite(payload, 1, len, gProto);
    std::fflush(gProto);
}

void sendText(int level, const std::string& text) {
    std::vector<char> buf(4 + text.size());
    const std::uint32_t lv = std::uint32_t(level);
    std::memcpy(buf.data(), &lv, 4);
    std::memcpy(buf.data() + 4, text.data(), text.size());
    send(4, buf.data(), std::uint32_t(buf.size()));
}

struct Commands {
    std::mutex m;
    std::vector<std::string> q;
    std::atomic<bool> eof{false};
};

}  // namespace

int main(int argc, char** argv) {
    // Keep the real stdout for the protocol; route printf/cout to stderr.
#ifdef _WIN32
    const int protoFd = _dup(_fileno(stdout));
    _setmode(protoFd, _O_BINARY);
    _dup2(_fileno(stderr), _fileno(stdout));
    gProto = _fdopen(protoFd, "wb");
#else
    gProto = fdopen(dup(fileno(stdout)), "wb");
    dup2(fileno(stderr), fileno(stdout));
#endif
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    if (argc < 2) {
        sendText(2, "usage: runner <settings-file>");
        return 2;
    }
    std::string warnings;
    Settings settings = loadSettings(argv[1], warnings);
    if (!warnings.empty()) sendText(1, warnings);

    sim::SimWorld world(settings.sim);
    world.reset(settings.ballStartX, settings.ballStartY);
    Runtime rt(settings, world);

    rt.onText = [](int level, const std::string& text) { sendText(level, text); };
    rt.onLog = [](const std::string& name, double t, double v) {
        std::vector<char> buf(16 + name.size());
        std::memcpy(buf.data(), &t, 8);
        std::memcpy(buf.data() + 8, &v, 8);
        std::memcpy(buf.data() + 16, name.data(), name.size());
        send(3, buf.data(), std::uint32_t(buf.size()));
    };
    rt.onFrame = [&](const sim::Frame& f, const Detection& det) {
        std::vector<char> buf(16 + 8 + 1 + 8 + f.data.size());
        char* p = buf.data();
        const std::uint32_t hdr[4] = {std::uint32_t(f.index), std::uint32_t(f.width), std::uint32_t(f.height),
                                      std::uint32_t(f.format)};
        std::memcpy(p, hdr, 16);
        p += 16;
        const double t = world.time();
        std::memcpy(p, &t, 8);
        p += 8;
        *p++ = det.found ? 1 : 0;
        std::memcpy(p, &det.x, 4);
        std::memcpy(p + 4, &det.y, 4);
        p += 8;
        std::memcpy(p, f.data.data(), f.data.size());
        send(2, buf.data(), std::uint32_t(buf.size()));
    };

    Commands cmds;
    std::thread reader([&] {
        std::string line;
        while (std::getline(std::cin, line)) {
            std::lock_guard<std::mutex> lock(cmds.m);
            cmds.q.push_back(line);
        }
        cmds.eof = true;
    });
    reader.detach();

    sendText(0, "Simulation started.");
    using Clock = std::chrono::steady_clock;
    double speed = settings.speed;
    bool paused = false;
    auto wall0 = Clock::now();
    double sim0 = world.time();
    double nextState = 0.0;
    double rtf = 1.0;
    auto rtfWall = Clock::now();
    double rtfSim = world.time();
    const double tick = 0.002;  // sim time between command/pacing checks
    const auto& geo = settings.sim.geometry;

    for (;;) {
        {
            std::vector<std::string> q;
            {
                std::lock_guard<std::mutex> lock(cmds.m);
                q.swap(cmds.q);
            }
            for (const auto& c : q) {
                std::istringstream is(c);
                std::string op;
                is >> op;
                if (op == "quit") return 0;
                if (op == "pause") paused = true;
                if (op == "resume") paused = false;
                if (op == "speed") is >> speed;
                if (op == "target") {
                    double x = 0, y = 0;
                    is >> x >> y;
                    rt.setTargetOverride(x * 1e-3, y * 1e-3);
                }
                if (op == "nudge") {
                    double vx = 0, vy = 0;
                    is >> vx >> vy;
                    world.nudgeBall(vx * 1e-3, vy * 1e-3);
                }
                if (op == "pause" || op == "resume" || op == "speed") {
                    wall0 = Clock::now();
                    sim0 = world.time();
                }
            }
            if (cmds.eof && q.empty()) {
                // app went away
                static int idle = 0;
                if (++idle > 100) return 0;
            }
        }

        if (!paused) {
            try {
                const double end = world.time() + tick;
                while (world.time() < end - 1e-12) {
                    world.step();
                    rt.update();
                }
            } catch (const std::exception& e) {
                sendText(2, std::string("Simulation stopped: ") + e.what());
                paused = true;
            }
            // pace to real time x speed
            const double simElapsed = world.time() - sim0;
            const double wallTarget = simElapsed / std::max(speed, 0.01);
            const double wallElapsed = std::chrono::duration<double>(Clock::now() - wall0).count();
            if (wallTarget > wallElapsed) {
                std::this_thread::sleep_for(std::chrono::duration<double>(wallTarget - wallElapsed));
            } else if (wallElapsed - wallTarget > 0.25) {
                wall0 = Clock::now();  // running behind: don't try to catch up
                sim0 = world.time();
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }

        const auto now = Clock::now();
        const double dw = std::chrono::duration<double>(now - rtfWall).count();
        if (dw > 0.5) {
            rtf = (world.time() - rtfSim) / dw;
            rtfWall = now;
            rtfSim = world.time();
        }

        if (world.time() >= nextState || paused) {
            nextState = world.time() + 1.0 / 60.0;
            const sim::GroundTruth g = world.truth();
            const Telemetry& te = rt.telemetry();
            double s[kStateSize] = {};
            s[S_T] = g.t;
            s[S_RTF] = paused ? 0.0 : rtf;
            s[S_PLATE_P + 0] = g.plate.p.x * 1e3;
            s[S_PLATE_P + 1] = g.plate.p.y * 1e3;
            s[S_PLATE_P + 2] = g.plate.p.z * 1e3;
            const Quat<double> q = Quat<double>::fromMatrix(g.plate.R);
            s[S_PLATE_Q + 0] = q.w;
            s[S_PLATE_Q + 1] = q.x;
            s[S_PLATE_Q + 2] = q.y;
            s[S_PLATE_Q + 3] = q.z;
            for (int i = 0; i < 3; ++i) {
                s[S_THETA + i] = g.theta[i];
                const Vec3<double> P = geo.crankPin(i, g.theta[i]);
                const Vec3<double> B = g.plate.toWorld(geo.jointInPlate(i));
                for (int k = 0; k < 3; ++k) {
                    s[S_PIN + 3 * i + k] = P[k] * 1e3;
                    s[S_JOINT + 3 * i + k] = B[k] * 1e3;
                }
            }
            s[S_BALL + 0] = g.ballWorld.x * 1e3;
            s[S_BALL + 1] = g.ballWorld.y * 1e3;
            s[S_BALL + 2] = g.ballWorld.z * 1e3;
            s[S_BALL_PLATE + 0] = g.ballPlate.x * 1e3;
            s[S_BALL_PLATE + 1] = g.ballPlate.y * 1e3;
            s[S_MEAS_FOUND] = te.measured ? 1.0 : 0.0;
            s[S_MEAS + 0] = te.measX * 1e3;
            s[S_MEAS + 1] = te.measY * 1e3;
            s[S_TARGET + 0] = te.targetX * 1e3;
            s[S_TARGET + 1] = te.targetY * 1e3;
            s[S_CMD + 0] = te.cmdAngleX;
            s[S_CMD + 1] = te.cmdAngleY;
            s[S_SLOPE + 0] = g.slopeX * 180.0 / 3.14159265358979323846;
            s[S_SLOPE + 1] = g.slopeY * 180.0 / 3.14159265358979323846;
            s[S_MODE] = double(int(g.mode));
            s[S_CPU_MS] = te.cpuTime * 1e3;
            s[S_PAUSED] = paused ? 1.0 : 0.0;
            s[S_HOST_MS] = te.hostTime * 1e3;
            send(1, s, sizeof(s));
        }
    }
}
