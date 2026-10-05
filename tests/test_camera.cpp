// Verification of the camera model and the renderer.
#include <cstdint>

#include "bbp/sim/renderer.hpp"
#include "test.hpp"

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

double luma(const Frame& f, int x, int y) {
    std::uint8_t r, g, b;
    f.rgb(x, y, r, g, b);
    return 0.299 * r + 0.587 * g + 0.114 * b;
}
}  // namespace

TEST(Camera_DistortionInverts) {
    SimParams p;
    auto intr = CameraIntrinsics<double>::fromHfov(320, 240, p.camera.pinholeHfovDeg * kDeg);
    intr.k1 = p.camera.k1;
    intr.k2 = p.camera.k2;
    double worst = 0.0;
    for (double u = 0.0; u <= 320.0; u += 8.0)
        for (double v = 0.0; v <= 240.0; v += 8.0) {
            double x, y, u2, v2;
            intr.pixelToNormalized(u, v, x, y);
            intr.normalizedToPixel(x, y, u2, v2);
            worst = std::max(worst, std::hypot(u2 - u, v2 - v));
        }
    std::printf("    max pixel round-trip error %.2e px\n", worst);
    // effective field of view including barrel distortion
    double x0, y0, x1, y1;
    intr.pixelToNormalized(0.0, 120.0, x0, y0);
    intr.pixelToNormalized(320.0, 120.0, x1, y1);
    std::printf("    effective horizontal FOV %.1f deg (pinhole model %.1f deg)\n",
                (std::atan(x1) - std::atan(x0)) / kDeg, p.camera.pinholeHfovDeg);
    CHECK(worst < 1e-6);
}

TEST(Camera_PlateRefractionLateralShift) {
    // A ray crossing a plane-parallel slab of thickness t at incidence i is displaced
    // sideways by t sin(i - r) / cos(r), with sin i = n sin r (Snell).
    SimParams p;
    CameraRenderer ren(p);
    const auto& cam = ren.geometry();
    Pose<double> plate;
    plate.p = Vec3d{0, 0, 0.040};
    const double t = cam.plateThickness(), n = cam.plateIndex();
    bbptest::Csv csv("cam_slab_shift.csv", "incidence_deg,shift_mm,analytic_mm");
    double worst = 0.0;
    for (double inc = 0.0; inc <= 45.0; inc += 5.0) {
        const Vec3d d{std::sin(inc * kDeg), 0.0, std::cos(inc * kDeg)};
        Vec3d exitPt;
        bool hit;
        CHECK(cam.traceThroughPlate(d, plate, 1.0, exitPt, hit));
        const Vec3d straight = cam.centre() + d * ((plate.p - cam.centre()).dot(Vec3d{0, 0, 1}) / d.z);
        const double shift = (exitPt - straight).cross(d).norm();
        const double r = std::asin(std::sin(inc * kDeg) / n);
        const double analytic = t * std::sin(inc * kDeg - r) / std::cos(r);
        worst = std::max(worst, std::fabs(shift - analytic));
        csv.row(inc, shift * 1e3, analytic * 1e3);
    }
    std::printf("    max deviation from analytic shift %.2e m\n", worst);
    CHECK(worst < 1e-12);
}

TEST(Camera_ProjectionBackprojectionRoundTrip) {
    SimParams p;
    CameraRenderer ren(p);
    const auto& cam = ren.geometry();
    const Kinematics<double> kin(p.geometry);
    const Pose<double> plate = kin.poseFromNormal(Kinematics<double>::normalFromSlopes(6 * kDeg, -4 * kDeg),
                                                  kin.levelHeight(0.0));
    double worst = 0.0;
    for (double x = -0.08; x <= 0.08; x += 0.02)
        for (double y = -0.08; y <= 0.08; y += 0.02) {
            const Vec3d X = plate.toWorld(Vec3d{x, y, p.ball.radius});
            double u, v;
            CHECK(cam.projectThroughPlate(X, plate, p.geometry.plateRadius, u, v));
            Vec3d back;
            CHECK(cam.backprojectToPlate(u, v, plate, p.geometry.plateRadius, p.ball.radius, back));
            worst = std::max(worst, std::hypot(back.x - x, back.y - y));
        }
    std::printf("    max round-trip error on the plate %.2e m\n", worst);
    CHECK(worst < 1e-9);
}

TEST(Camera_RenderedBallCentroidMatchesProjection) {
    // Noise-free render of a static scene. The centroid of the dark ball blob, back-projected
    // through the plate, should land on the true ball position; the residual is the
    // perspective bias of a sphere's silhouette (largest off-axis).
    SimParams p;
    p.sensor.readNoise = 0.0;
    p.sensor.fullWell = 1e12;  // makes shot noise negligible
    p.camera.temporalSamples = 1;
    p.camera.spatialSamples = 4;
    CameraRenderer ren(p);
    const auto& cam = ren.geometry();
    const Kinematics<double> kin(p.geometry);
    const Pose<double> plate = kin.poseFromNormal(Vec3d{0, 0, 1}, kin.levelHeight(0.0));
    bbptest::Csv csv("cam_centroid.csv", "x_mm,y_mm,radius_mm,err_mm,err_px");
    double worstMm = 0.0;
    for (double rad : {0.0, 0.02, 0.04, 0.06, 0.08}) {
        const Vec3d ballPlate{rad * std::cos(0.4), rad * std::sin(0.4), p.ball.radius};
        SceneState sc{plate, plate.toWorld(ballPlate), true};
        Frame f, bgf;
        ren.render([&](double) { return sc; }, 0.0, 1, f);
        SceneState empty = sc;
        empty.ballPresent = false;
        ren.render([&](double) { return empty; }, 0.0, 1, bgf);
        if (rad == 0.04 && !bbptest::csvDir.empty()) writePpm(bbptest::csvDir + "/cam_frame.ppm", f);
        double u0, v0;
        cam.projectThroughPlate(sc.ball, plate, p.geometry.plateRadius, u0, v0);
        // weight = darkening relative to the same scene without the ball
        double sw = 0.0, su = 0.0, sv = 0.0;
        for (int y = std::max(0, int(v0) - 20); y < std::min(f.height, int(v0) + 20); ++y)
            for (int x = std::max(0, int(u0) - 20); x < std::min(f.width, int(u0) + 20); ++x) {
                const double w = std::max(0.0, luma(bgf, x, y) - luma(f, x, y));
                sw += w;
                su += w * (x + 0.5);
                sv += w * (y + 0.5);
            }
        Vec3d est;
        cam.backprojectToPlate(su / sw, sv / sw, plate, p.geometry.plateRadius, p.ball.radius, est);
        const double errMm = std::hypot(est.x - ballPlate.x, est.y - ballPlate.y) * 1e3;
        const double errPx = std::hypot(su / sw - u0, sv / sw - v0);
        std::printf("    ball at r = %2.0f mm: centroid error %.3f mm (%.3f px)\n", rad * 1e3, errMm, errPx);
        csv.row(ballPlate.x * 1e3, ballPlate.y * 1e3, rad * 1e3, errMm, errPx);
        worstMm = std::max(worstMm, errMm);
    }
    CHECK(worstMm < 1.5);
}
