// Small fixed-size linear algebra used by the shared core (firmware and simulator).
// Header-only, no heap allocation, no exceptions, so it builds for the ESP32-S3 as well
// as for the desktop. Scalar type T is float on the target and double in the simulator.
#pragma once

#include <cmath>

namespace bbp {

template <typename T>
struct Vec3 {
    T x{}, y{}, z{};

    constexpr Vec3() = default;
    constexpr Vec3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
    template <typename U>
    constexpr explicit Vec3(const Vec3<U>& o) : x(T(o.x)), y(T(o.y)), z(T(o.z)) {}

    constexpr T& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    constexpr T operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(T s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(T s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(T s) { x *= s; y *= s; z *= s; return *this; }

    constexpr T dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    constexpr Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    T norm() const { return std::sqrt(dot(*this)); }
    constexpr T squaredNorm() const { return dot(*this); }
    Vec3 normalized() const {
        T n = norm();
        return n > T(0) ? *this / n : Vec3{};
    }
};

template <typename T>
constexpr Vec3<T> operator*(T s, const Vec3<T>& v) { return v * s; }

template <typename T>
struct Mat3 {
    T m[3][3]{};

    static constexpr Mat3 identity() {
        Mat3 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = T(1);
        return r;
    }
    static constexpr Mat3 fromColumns(const Vec3<T>& a, const Vec3<T>& b, const Vec3<T>& c) {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            r.m[i][0] = a[i];
            r.m[i][1] = b[i];
            r.m[i][2] = c[i];
        }
        return r;
    }
    static constexpr Mat3 diagonal(T a, T b, T c) {
        Mat3 r;
        r.m[0][0] = a;
        r.m[1][1] = b;
        r.m[2][2] = c;
        return r;
    }
    template <typename U>
    static constexpr Mat3 cast(const Mat3<U>& o) {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = T(o.m[i][j]);
        return r;
    }

    constexpr Vec3<T> col(int j) const { return {m[0][j], m[1][j], m[2][j]}; }
    constexpr Vec3<T> operator*(const Vec3<T>& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    constexpr Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
        return r;
    }
    constexpr Mat3 operator+(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][j] + o.m[i][j];
        return r;
    }
    constexpr Mat3 operator-(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][j] - o.m[i][j];
        return r;
    }
    constexpr Mat3 operator*(T s) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][j] * s;
        return r;
    }
    constexpr Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
        return r;
    }
};

// Cross-product matrix: skew(a) * b == a.cross(b).
template <typename T>
constexpr Mat3<T> skew(const Vec3<T>& a) {
    Mat3<T> r;
    r.m[0][1] = -a.z; r.m[0][2] = a.y;
    r.m[1][0] = a.z;  r.m[1][2] = -a.x;
    r.m[2][0] = -a.y; r.m[2][1] = a.x;
    return r;
}

// Rotation matrix exp(skew(w)) via the Rodrigues formula.
template <typename T>
Mat3<T> rotationFromRotationVector(const Vec3<T>& w) {
    const T angle = w.norm();
    const Mat3<T> K = skew(w);
    if (angle < T(1e-6)) {
        // second-order series, accurate to O(angle^3)
        return Mat3<T>::identity() + K + K * K * T(0.5);
    }
    const T a = std::sin(angle) / angle;
    const T b = (T(1) - std::cos(angle)) / (angle * angle);
    return Mat3<T>::identity() + K * a + K * K * b;
}

template <typename T>
Mat3<T> rotationZ(T a) {
    const T c = std::cos(a), s = std::sin(a);
    Mat3<T> r = Mat3<T>::identity();
    r.m[0][0] = c; r.m[0][1] = -s;
    r.m[1][0] = s; r.m[1][1] = c;
    return r;
}

template <typename T>
Mat3<T> rotationY(T a) {
    const T c = std::cos(a), s = std::sin(a);
    Mat3<T> r = Mat3<T>::identity();
    r.m[0][0] = c;  r.m[0][2] = s;
    r.m[2][0] = -s; r.m[2][2] = c;
    return r;
}

// Re-orthonormalise a rotation matrix (Gram-Schmidt on the columns) to remove drift.
template <typename T>
Mat3<T> orthonormalized(const Mat3<T>& R) {
    Vec3<T> x = R.col(0).normalized();
    Vec3<T> y = R.col(1);
    y = (y - x * x.dot(y)).normalized();
    Vec3<T> z = x.cross(y);
    return Mat3<T>::fromColumns(x, y, z);
}

// Unit quaternion (w, x, y, z), used to interpolate orientations.
template <typename T>
struct Quat {
    T w{1}, x{}, y{}, z{};

    static Quat fromMatrix(const Mat3<T>& R) {
        Quat q;
        const T tr = R.m[0][0] + R.m[1][1] + R.m[2][2];
        if (tr > T(0)) {
            T s = std::sqrt(tr + T(1)) * T(2);
            q.w = T(0.25) * s;
            q.x = (R.m[2][1] - R.m[1][2]) / s;
            q.y = (R.m[0][2] - R.m[2][0]) / s;
            q.z = (R.m[1][0] - R.m[0][1]) / s;
        } else if (R.m[0][0] > R.m[1][1] && R.m[0][0] > R.m[2][2]) {
            T s = std::sqrt(T(1) + R.m[0][0] - R.m[1][1] - R.m[2][2]) * T(2);
            q.w = (R.m[2][1] - R.m[1][2]) / s;
            q.x = T(0.25) * s;
            q.y = (R.m[0][1] + R.m[1][0]) / s;
            q.z = (R.m[0][2] + R.m[2][0]) / s;
        } else if (R.m[1][1] > R.m[2][2]) {
            T s = std::sqrt(T(1) + R.m[1][1] - R.m[0][0] - R.m[2][2]) * T(2);
            q.w = (R.m[0][2] - R.m[2][0]) / s;
            q.x = (R.m[0][1] + R.m[1][0]) / s;
            q.y = T(0.25) * s;
            q.z = (R.m[1][2] + R.m[2][1]) / s;
        } else {
            T s = std::sqrt(T(1) + R.m[2][2] - R.m[0][0] - R.m[1][1]) * T(2);
            q.w = (R.m[1][0] - R.m[0][1]) / s;
            q.x = (R.m[0][2] + R.m[2][0]) / s;
            q.y = (R.m[1][2] + R.m[2][1]) / s;
            q.z = T(0.25) * s;
        }
        return q.normalized();
    }

    Quat normalized() const {
        T n = std::sqrt(w * w + x * x + y * y + z * z);
        return {w / n, x / n, y / n, z / n};
    }

    Mat3<T> toMatrix() const {
        Mat3<T> R;
        R.m[0][0] = 1 - 2 * (y * y + z * z); R.m[0][1] = 2 * (x * y - z * w);     R.m[0][2] = 2 * (x * z + y * w);
        R.m[1][0] = 2 * (x * y + z * w);     R.m[1][1] = 1 - 2 * (x * x + z * z); R.m[1][2] = 2 * (y * z - x * w);
        R.m[2][0] = 2 * (x * z - y * w);     R.m[2][1] = 2 * (y * z + x * w);     R.m[2][2] = 1 - 2 * (x * x + y * y);
        return R;
    }

    // Normalised linear interpolation; adequate for the sub-millisecond spans it is used on.
    static Quat nlerp(const Quat& a, Quat b, T s) {
        if (a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z < T(0)) b = {-b.w, -b.x, -b.y, -b.z};
        return Quat{a.w + (b.w - a.w) * s, a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s,
                    a.z + (b.z - a.z) * s}.normalized();
    }
};

// Solve A x = b in place for small dense N x N systems (Gaussian elimination with partial
// pivoting). On return b holds x. Returns false if A is numerically singular.
template <typename T, int N>
bool solveLinear(T (&A)[N][N], T (&b)[N]) {
    for (int k = 0; k < N; ++k) {
        int p = k;
        T best = std::fabs(A[k][k]);
        for (int i = k + 1; i < N; ++i) {
            if (std::fabs(A[i][k]) > best) {
                best = std::fabs(A[i][k]);
                p = i;
            }
        }
        if (best < T(1e-12)) return false;
        if (p != k) {
            for (int j = 0; j < N; ++j) {
                T t = A[k][j]; A[k][j] = A[p][j]; A[p][j] = t;
            }
            T t = b[k]; b[k] = b[p]; b[p] = t;
        }
        for (int i = k + 1; i < N; ++i) {
            const T f = A[i][k] / A[k][k];
            for (int j = k; j < N; ++j) A[i][j] -= f * A[k][j];
            b[i] -= f * b[k];
        }
    }
    for (int i = N - 1; i >= 0; --i) {
        T s = b[i];
        for (int j = i + 1; j < N; ++j) s -= A[i][j] * b[j];
        b[i] = s / A[i][i];
    }
    return true;
}

template <typename T>
constexpr T kPi = T(3.14159265358979323846);

template <typename T>
constexpr T deg2rad(T d) { return d * kPi<T> / T(180); }

template <typename T>
constexpr T rad2deg(T r) { return r * T(180) / kPi<T>; }

}  // namespace bbp
