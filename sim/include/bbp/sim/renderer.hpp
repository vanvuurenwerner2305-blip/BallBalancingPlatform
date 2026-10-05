// Physically based image formation for the under-plate camera: ray casting through the
// refracting plate, ball shading, rolling-shutter timing, motion blur, sensor noise and the
// output pixel format the ESP32 camera driver delivers.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "bbp/core/camera_geometry.hpp"
#include "bbp/sim/ball.hpp"
#include "bbp/sim/params.hpp"

namespace bbp::sim {

struct Frame {
    int width = 0, height = 0;
    PixelFormat format = PixelFormat::RGB565;
    std::vector<std::uint8_t> data;  // RGB565 is big-endian, as delivered by esp32-camera
    std::uint64_t index = 0;
    double exposureStart = 0.0;      // start of row 0's exposure [s]
    double readyTime = 0.0;          // when the frame is handed to the application [s]

    // Decoded 8-bit RGB of pixel (x, y), whatever the storage format.
    void rgb(int x, int y, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b) const;
};

// What the camera can see at one instant.
struct SceneState {
    Posed plate;
    Vec3d ball;
    bool ballPresent = true;
};

class CameraRenderer {
public:
    using SceneFn = std::function<SceneState(double t)>;

    explicit CameraRenderer(const SimParams& p);

    const CameraGeometry<double>& geometry() const { return cam_; }

    // Render the frame whose row 0 starts exposing at `exposureStart`.
    void render(const SceneFn& scene, double exposureStart, std::uint64_t index, Frame& out) const;

    // Time at which the last row of a frame starting at `exposureStart` finishes exposing.
    double exposureEnd(double exposureStart) const;
    double rowTime() const { return rowTime_; }

    // Noise-free linear radiance (RGB) along one camera ray; exposed for verification.
    void radiance(const Vec3d& dir, const SceneState& s, double (&rgb)[3]) const;

private:
    void sky(const Vec3d& d, double (&rgb)[3]) const;
    void shadeBall(const Vec3d& hit, const Vec3d& normal, const Vec3d& viewDir, double (&rgb)[3]) const;
    double slabTransmittance(double cosIncidence) const;
    void renderRows(const SceneFn& scene, double exposureStart, std::uint64_t index, int y0, int y1,
                    Frame& out) const;

    SimParams p_;
    CameraGeometry<double> cam_;
    double rowTime_;
    int S_, St_;
    std::vector<Vec3d> rays_;        // per pixel per sub-sample, world directions
    std::vector<float> vignette_;    // per pixel relative illumination
    double ballRadius_;
    Vec3d bosses_[3];                // plate-frame joint boss centres (on the bottom face)
};

}  // namespace bbp::sim
