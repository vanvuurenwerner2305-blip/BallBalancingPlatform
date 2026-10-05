#include "bbp/sim/renderer.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

#include "bbp/sim/rng.hpp"

namespace bbp::sim {

namespace {
constexpr double kPiD = 3.14159265358979323846;
constexpr double kDeg = kPiD / 180.0;

// Unpolarised Fresnel reflectance at an interface n1 -> n2 for incidence cosine cosi.
double fresnelReflectance(double n1, double n2, double cosi) {
    const double sint2 = (n1 / n2) * (n1 / n2) * (1.0 - cosi * cosi);
    if (sint2 >= 1.0) return 1.0;
    const double cost = std::sqrt(1.0 - sint2);
    const double rs = (n1 * cosi - n2 * cost) / (n1 * cosi + n2 * cost);
    const double rp = (n1 * cost - n2 * cosi) / (n1 * cost + n2 * cosi);
    return 0.5 * (rs * rs + rp * rp);
}
}  // namespace

void Frame::rgb(int x, int y, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b) const {
    const std::size_t i = std::size_t(y) * width + x;
    switch (format) {
        case PixelFormat::Gray8:
            r = g = b = data[i];
            break;
        case PixelFormat::RGB565: {
            const std::uint16_t v = std::uint16_t((data[2 * i] << 8) | data[2 * i + 1]);
            const int r5 = (v >> 11) & 31, g6 = (v >> 5) & 63, b5 = v & 31;
            r = std::uint8_t((r5 * 255 + 15) / 31);
            g = std::uint8_t((g6 * 255 + 31) / 63);
            b = std::uint8_t((b5 * 255 + 15) / 31);
            break;
        }
        case PixelFormat::RGB888:
            r = data[3 * i];
            g = data[3 * i + 1];
            b = data[3 * i + 2];
            break;
    }
}

CameraRenderer::CameraRenderer(const SimParams& p) : p_(p) {
    const auto& c = p.camera;
    auto intr = CameraIntrinsics<double>::fromHfov(c.width, c.height, c.pinholeHfovDeg * kDeg);
    intr.k1 = c.k1;
    intr.k2 = c.k2;
    intr.k3 = c.k3;
    intr.p1 = c.p1;
    intr.p2 = c.p2;
    cam_ = CameraGeometry<double>(intr, Vec3d{c.x, c.y, c.z}, c.yawDeg * kDeg, p.geometry.plateThickness,
                                  p.plate.refractiveIndex);
    rowTime_ = c.readoutTime / c.height;
    S_ = std::max(1, c.spatialSamples);
    St_ = std::max(1, c.temporalSamples);
    rays_.resize(std::size_t(c.width) * c.height * S_ * S_);
    vignette_.resize(std::size_t(c.width) * c.height);
    for (int y = 0; y < c.height; ++y) {
        for (int x = 0; x < c.width; ++x) {
            for (int sy = 0; sy < S_; ++sy)
                for (int sx = 0; sx < S_; ++sx)
                    rays_[((std::size_t(y) * c.width + x) * S_ + sy) * S_ + sx] =
                        cam_.pixelRay(x + (sx + 0.5) / S_, y + (sy + 0.5) / S_);
            const Vec3d d = cam_.pixelRay(x + 0.5, y + 0.5);
            const double cosField = d.dot(cam_.rotation().col(2));
            vignette_[std::size_t(y) * c.width + x] = float(std::pow(cosField, p.sensor.vignettingPower));
        }
    }
    ballRadius_ = p.ball.radius;
    for (int i = 0; i < 3; ++i) {
        const Vec3d j = p.geometry.jointInPlate(i);
        bosses_[i] = Vec3d{j.x, j.y, -p.geometry.plateThickness};
    }
}

double CameraRenderer::exposureEnd(double exposureStart) const {
    return exposureStart + (p_.camera.height - 1) * rowTime_ + p_.camera.exposure;
}

double CameraRenderer::slabTransmittance(double cosi) const {
    // Incoherent sum over internal reflections of a plane-parallel absorbing slab:
    // T = (1-R)^2 tau / (1 - R^2 tau^2), tau = exp(-alpha * path length).
    const double n = p_.plate.refractiveIndex;
    const double R = fresnelReflectance(1.0, n, cosi);
    const double sint = std::sqrt(std::max(0.0, 1.0 - cosi * cosi)) / n;
    const double cost = std::sqrt(1.0 - sint * sint);
    const double tau = std::exp(-p_.plate.absorption * p_.geometry.plateThickness / cost);
    return (1.0 - R) * (1.0 - R) * tau / (1.0 - R * R * tau * tau);
}

void CameraRenderer::sky(const Vec3d& d, double (&rgb)[3]) const {
    const auto& L = p_.lighting;
    if (d.z > 0.0) {
        const double f = 1.0 + L.skyGradient * d.x;
        for (int k = 0; k < 3; ++k) rgb[k] = L.skyRadiance[k] * f;
    } else {
        for (int k = 0; k < 3; ++k) rgb[k] = L.baseRadiance[k];
    }
}

void CameraRenderer::shadeBall(const Vec3d& hit, const Vec3d& N, const Vec3d& V, double (&rgb)[3]) const {
    const auto& L = p_.lighting;
    // Hemisphere lighting: irradiance from a uniform upper (ceiling) and lower (base) hemisphere,
    // E(N) = pi [L_up (1 + N_z)/2 + L_down (1 - N_z)/2]; Lambertian radiance = albedo E / pi.
    const double up = 0.5 * (1.0 + N.z), down = 0.5 * (1.0 - N.z);
    for (int k = 0; k < 3; ++k)
        rgb[k] = L.ballAlbedo[k] * (L.skyRadiance[k] * up + L.baseRadiance[k] * down);
    // Mirror reflection of the environment.
    const Vec3d viewIn = V * -1.0;
    const Vec3d R = viewIn - N * (2.0 * viewIn.dot(N));
    double env[3];
    sky(R, env);
    for (int k = 0; k < 3; ++k) rgb[k] += L.ballMirror * env[k];
    // Optional ring light at the camera: Lambertian + normalised Blinn-Phong highlight.
    if (L.ledIntensity > 0.0) {
        Vec3d toLed = cam_.centre() - hit;
        const double dist2 = toLed.squaredNorm();
        toLed = toLed / std::sqrt(dist2);
        const double cosN = std::max(0.0, N.dot(toLed));
        const double E = L.ledIntensity * cosN / dist2;
        const Vec3d Hh = (toLed + V).normalized();
        const double spec = L.ballSpecular * (L.ballShininess + 8.0) / (8.0 * kPiD) *
                            std::pow(std::max(0.0, N.dot(Hh)), L.ballShininess);
        for (int k = 0; k < 3; ++k) rgb[k] += (L.ballAlbedo[k] / kPiD + spec) * E;
    }
}

void CameraRenderer::radiance(const Vec3d& d, const SceneState& s, double (&rgb)[3]) const {
    Vec3d origin = cam_.centre();
    double T = 1.0;
    Vec3d exitPoint;
    bool hitPlate = false;
    if (cam_.traceThroughPlate(d, s.plate, p_.geometry.plateRadius, exitPoint, hitPlate)) {
        if (hitPlate) {
            // Joint bosses on the underside are opaque.
            const Vec3d n = s.plate.normal();
            const Vec3d bottom = s.plate.p - n * p_.geometry.plateThickness;
            const Vec3d X1 = cam_.centre() + d * ((bottom - cam_.centre()).dot(n) / d.dot(n));
            const Vec3d q = s.plate.toPlate(X1);
            for (const Vec3d& b : bosses_) {
                const double dx = q.x - b.x, dy = q.y - b.y;
                if (dx * dx + dy * dy < p_.plate.bossRadius * p_.plate.bossRadius) {
                    for (int k = 0; k < 3; ++k) rgb[k] = p_.lighting.bossRadiance[k];
                    return;
                }
            }
            T = slabTransmittance(d.dot(n));
            origin = exitPoint;
        }
    }
    if (s.ballPresent) {
        const Vec3d oc = origin - s.ball;
        const double b = oc.dot(d);
        const double cc = oc.squaredNorm() - ballRadius_ * ballRadius_;
        const double disc = b * b - cc;
        if (disc > 0.0) {
            const double tHit = -b - std::sqrt(disc);
            if (tHit > 0.0) {
                const Vec3d hit = origin + d * tHit;
                const Vec3d N = (hit - s.ball) / ballRadius_;
                shadeBall(hit, N, d * -1.0, rgb);
                for (int k = 0; k < 3; ++k) rgb[k] *= T;
                return;
            }
        }
    }
    sky(d, rgb);
    for (int k = 0; k < 3; ++k) rgb[k] *= T;
}

void CameraRenderer::renderRows(const SceneFn& scene, double t0, std::uint64_t index, int y0, int y1,
                                Frame& out) const {
    const auto& c = p_.camera;
    const auto& sn = p_.sensor;
    const int W = c.width;
    const double expScale = sn.fullWell * sn.skyLevel * (c.exposure / sn.referenceExposure);
    std::vector<SceneState> states(St_);
    for (int y = y0; y < y1; ++y) {
        const double rowStart = t0 + y * rowTime_;
        for (int k = 0; k < St_; ++k) states[k] = scene(rowStart + (k + 0.5) / St_ * c.exposure);
        for (int x = 0; x < W; ++x) {
            double acc[3] = {0, 0, 0};
            const Vec3d* rp = &rays_[(std::size_t(y) * W + x) * S_ * S_];
            for (int k = 0; k < St_; ++k) {
                for (int s = 0; s < S_ * S_; ++s) {
                    double L[3];
                    radiance(rp[s], states[k], L);
                    acc[0] += L[0];
                    acc[1] += L[1];
                    acc[2] += L[2];
                }
            }
            const double norm = 1.0 / (St_ * S_ * S_);
            const double vig = vignette_[std::size_t(y) * W + x];
            // Poisson-Gaussian sensor model (Foi et al. 2008): photo-electrons with shot noise
            // (Gaussian approximation) plus read noise, clipped at full well.
            Rng rng(p_.numerics.seed, (index << 24) ^ (std::uint64_t(y) * W + x));
            std::uint8_t q[3];
            for (int ch = 0; ch < 3; ++ch) {
                const double mu = acc[ch] * norm * vig * expScale;
                double e = mu + std::sqrt(std::max(mu, 0.0)) * rng.normal() + sn.readNoise * rng.normal();
                e = std::clamp(e / sn.fullWell, 0.0, 1.0);
                q[ch] = std::uint8_t(std::lround(255.0 * std::pow(e, 1.0 / sn.gamma)));
            }
            const std::size_t i = std::size_t(y) * W + x;
            switch (c.format) {
                case PixelFormat::Gray8:
                    out.data[i] = std::uint8_t(std::lround(0.299 * q[0] + 0.587 * q[1] + 0.114 * q[2]));
                    break;
                case PixelFormat::RGB565: {
                    const std::uint16_t v = std::uint16_t(((q[0] >> 3) << 11) | ((q[1] >> 2) << 5) | (q[2] >> 3));
                    out.data[2 * i] = std::uint8_t(v >> 8);
                    out.data[2 * i + 1] = std::uint8_t(v & 0xff);
                    break;
                }
                case PixelFormat::RGB888:
                    out.data[3 * i] = q[0];
                    out.data[3 * i + 1] = q[1];
                    out.data[3 * i + 2] = q[2];
                    break;
            }
        }
    }
}

void CameraRenderer::render(const SceneFn& scene, double exposureStart, std::uint64_t index, Frame& out) const {
    const auto& c = p_.camera;
    out.width = c.width;
    out.height = c.height;
    out.format = c.format;
    out.index = index;
    out.exposureStart = exposureStart;
    out.readyTime = exposureEnd(exposureStart) + c.latency;
    const int bpp = c.format == PixelFormat::Gray8 ? 1 : (c.format == PixelFormat::RGB565 ? 2 : 3);
    out.data.assign(std::size_t(c.width) * c.height * bpp, 0);

    int nThreads = p_.numerics.renderThreads;
    if (nThreads <= 0) nThreads = int(std::max(1u, std::thread::hardware_concurrency()));
    nThreads = std::min(nThreads, c.height);
    if (nThreads == 1) {
        renderRows(scene, exposureStart, index, 0, c.height, out);
        return;
    }
    std::vector<std::thread> pool;
    for (int t = 0; t < nThreads; ++t) {
        const int y0 = c.height * t / nThreads, y1 = c.height * (t + 1) / nThreads;
        pool.emplace_back([&, y0, y1] { renderRows(scene, exposureStart, index, y0, y1, out); });
    }
    for (auto& th : pool) th.join();
}

}  // namespace bbp::sim
