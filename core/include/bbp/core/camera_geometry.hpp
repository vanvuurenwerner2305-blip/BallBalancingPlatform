// Camera geometry shared by the firmware and the simulator: pinhole model with
// Brown-Conrady distortion, plus refraction through the transparent plate the camera looks
// through. The firmware uses back-projection to turn a pixel into a position on the plate;
// the simulator uses the same model to render images, so both sides agree by construction.
#pragma once

#include "bbp/core/kinematics.hpp"
#include "bbp/core/math.hpp"

namespace bbp {

template <typename T>
struct CameraIntrinsics {
    int width = 320, height = 240;
    T fx{}, fy{}, cx{}, cy{};
    T k1{}, k2{}, k3{}, p1{}, p2{};

    // Square pixels, principal point at the image centre, focal length from the horizontal
    // field of view of the undistorted pinhole model.
    static CameraIntrinsics fromHfov(int w, int h, T hfovRad) {
        CameraIntrinsics c;
        c.width = w;
        c.height = h;
        c.fx = c.fy = (T(w) / T(2)) / std::tan(hfovRad / T(2));
        c.cx = T(w) / T(2);
        c.cy = T(h) / T(2);
        return c;
    }

    // Normalised undistorted coordinates -> normalised distorted coordinates.
    void distort(T x, T y, T& xd, T& yd) const {
        const T r2 = x * x + y * y;
        const T radial = T(1) + r2 * (k1 + r2 * (k2 + r2 * k3));
        xd = x * radial + T(2) * p1 * x * y + p2 * (r2 + T(2) * x * x);
        yd = y * radial + p1 * (r2 + T(2) * y * y) + T(2) * p2 * x * y;
    }

    // Inverse of distort() by fixed-point iteration (as in OpenCV's undistortPoints).
    void undistort(T xd, T yd, T& x, T& y) const {
        x = xd;
        y = yd;
        for (int i = 0; i < 30; ++i) {
            const T r2 = x * x + y * y;
            const T radial = T(1) + r2 * (k1 + r2 * (k2 + r2 * k3));
            const T dx = T(2) * p1 * x * y + p2 * (r2 + T(2) * x * x);
            const T dy = p1 * (r2 + T(2) * y * y) + T(2) * p2 * x * y;
            x = (xd - dx) / radial;
            y = (yd - dy) / radial;
        }
    }

    // Pixel coordinates use the convention that pixel (i, j) covers [i, i+1) x [j, j+1),
    // so its centre is at (i + 0.5, j + 0.5).
    void pixelToNormalized(T u, T v, T& x, T& y) const {
        undistort((u - cx) / fx, (v - cy) / fy, x, y);
    }
    void normalizedToPixel(T x, T y, T& u, T& v) const {
        T xd, yd;
        distort(x, y, xd, yd);
        u = fx * xd + cx;
        v = fy * yd + cy;
    }
};

// Refraction of unit direction d at a surface with unit normal n facing the incoming ray
// (n.d < 0), going from index n1 to n2 (eta = n1/n2). Returns false on total internal
// reflection.
template <typename T>
bool refract(const Vec3<T>& d, const Vec3<T>& n, T eta, Vec3<T>& out) {
    const T cosi = -n.dot(d);
    const T sin2t = eta * eta * (T(1) - cosi * cosi);
    if (sin2t > T(1)) return false;
    out = d * eta + n * (eta * cosi - std::sqrt(T(1) - sin2t));
    return true;
}

template <typename T>
class CameraGeometry {
public:
    CameraGeometry() = default;
    // Camera looking along world +z from `centre`, rotated by `yaw` about its optical axis.
    CameraGeometry(const CameraIntrinsics<T>& intr, const Vec3<T>& centre, T yaw, T plateThickness,
                   T plateIndex)
        : intr_(intr), C_(centre), thickness_(plateThickness), index_(plateIndex) {
        const Vec3<T> xc{std::cos(yaw), std::sin(yaw), T(0)};
        const Vec3<T> zc{T(0), T(0), T(1)};
        R_ = Mat3<T>::fromColumns(xc, zc.cross(xc), zc);
    }

    const CameraIntrinsics<T>& intrinsics() const { return intr_; }
    const Vec3<T>& centre() const { return C_; }
    const Mat3<T>& rotation() const { return R_; }  // camera -> world
    T plateThickness() const { return thickness_; }
    T plateIndex() const { return index_; }

    // World-frame unit direction of the ray through pixel coordinates (u, v).
    Vec3<T> pixelRay(T u, T v) const {
        T x, y;
        intr_.pixelToNormalized(u, v, x, y);
        return (R_ * Vec3<T>{x, y, T(1)}).normalized();
    }

    // Projection of a world point ignoring the plate (straight ray). False if behind camera.
    bool projectDirect(const Vec3<T>& X, T& u, T& v) const {
        const Vec3<T> Xc = R_.transposed() * (X - C_);
        if (Xc.z <= T(0)) return false;
        intr_.normalizedToPixel(Xc.x / Xc.z, Xc.y / Xc.z, u, v);
        return true;
    }

    // Trace a camera ray through the plate (thickness and index given at construction; the
    // plate pose is `plate`, whose origin is the centre of its top face). On success `exitPoint`
    // is where the ray leaves the top face and the returned direction is unchanged (plane-parallel
    // slab). `hitPlate` reports whether the ray passed through the plate disc of `plateRadius`.
    bool traceThroughPlate(const Vec3<T>& d, const Pose<T>& plate, T plateRadius, Vec3<T>& exitPoint,
                           bool& hitPlate) const {
        const Vec3<T> n = plate.normal();
        const T dn = d.dot(n);
        if (dn <= T(1e-9)) return false;
        const Vec3<T> bottom = plate.p - n * thickness_;
        const T s1 = (bottom - C_).dot(n) / dn;
        if (s1 <= T(0)) return false;
        const Vec3<T> X1 = C_ + d * s1;
        const Vec3<T> q = plate.R.transposed() * (X1 - plate.p);
        hitPlate = (q.x * q.x + q.y * q.y) <= plateRadius * plateRadius;
        if (!hitPlate) {
            exitPoint = C_ + d * ((plate.p - C_).dot(n) / dn);
            return true;
        }
        Vec3<T> d1;
        if (!refract(d, -n, T(1) / index_, d1)) return false;
        exitPoint = X1 + d1 * (thickness_ / d1.dot(n));
        return true;
    }

    // Back-project pixel (u, v) onto the plane parallel to the plate at `height` above its top
    // face (for the ball centre: height = ball radius). Returns plate-frame coordinates.
    bool backprojectToPlate(T u, T v, const Pose<T>& plate, T plateRadius, T height, Vec3<T>& xPlate) const {
        const Vec3<T> d = pixelRay(u, v);
        Vec3<T> X2;
        bool hit;
        if (!traceThroughPlate(d, plate, plateRadius, X2, hit)) return false;
        const Vec3<T> n = plate.normal();
        const T dn = d.dot(n);
        const Vec3<T> X3 = X2 + d * ((plate.p + n * height - X2).dot(n) / dn);
        xPlate = plate.toPlate(X3);
        return true;
    }

    // Pixel at which a world point above the plate appears when seen through the plate.
    // Solved by fixed-point iteration on the back-projection (converges in a few steps
    // because refraction is a small perturbation of the straight ray).
    bool projectThroughPlate(const Vec3<T>& X, const Pose<T>& plate, T plateRadius, T& u, T& v) const {
        if (!projectDirect(X, u, v)) return false;
        const Vec3<T> target = plate.toPlate(X);
        const T height = target.z;
        for (int it = 0; it < 20; ++it) {
            Vec3<T> got;
            if (!backprojectToPlate(u, v, plate, plateRadius, height, got)) return false;
            const Vec3<T> errW = plate.R * Vec3<T>{target.x - got.x, target.y - got.y, T(0)};
            // move the pixel by the direct-projection sensitivity
            T u2, v2;
            if (!projectDirect(plate.toWorld(got) + errW, u2, v2)) return false;
            T u1, v1;
            projectDirect(plate.toWorld(got), u1, v1);
            u += u2 - u1;
            v += v2 - v1;
            if (std::fabs(u2 - u1) + std::fabs(v2 - v1) < T(1e-6)) break;
        }
        return true;
    }

private:
    CameraIntrinsics<T> intr_{};
    Vec3<T> C_{};
    Mat3<T> R_ = Mat3<T>::identity();
    T thickness_{};
    T index_{T(1)};
};

}  // namespace bbp
