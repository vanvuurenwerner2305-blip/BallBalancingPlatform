#include "bbp/sim/ball.hpp"

#include <algorithm>
#include <cmath>

namespace bbp::sim {

namespace {
constexpr double kPiD = 3.14159265358979323846;

Vec3d tangentialPart(const Vec3d& x, const Vec3d& n) { return x - n * x.dot(n); }
}  // namespace

const char* toString(ContactMode m) {
    switch (m) {
        case ContactMode::Rolling: return "rolling";
        case ContactMode::Stuck: return "stuck";
        case ContactMode::Slipping: return "slipping";
        case ContactMode::Flight: return "flight";
        case ContactMode::Lost: return "lost";
    }
    return "?";
}

double BallParams::mass() const {
    return density * 4.0 / 3.0 * kPiD * (radius * radius * radius - innerRadius * innerRadius * innerRadius);
}

double BallParams::inertiaFactor() const {
    const double a = radius, b = innerRadius;
    if (b <= 0.0) return 0.4;
    // Thick spherical shell: I = (2/5) m (a^5 - b^5) / (a^3 - b^3)
    return 0.4 * (std::pow(a, 5) - std::pow(b, 5)) / ((a * a * a - b * b * b) * a * a);
}

BallModel::BallModel(const BallParams& ball, const ContactParams& contact, const PlateParams& plate,
                     const EnvironmentParams& env, double plateRadius)
    : ball_(ball), c_(contact), env_(env), plateRadius_(plateRadius) {
    m_ = ball.mass();
    a_ = ball.radius;
    k_ = ball.inertiaFactor();
    I_ = k_ * m_ * a_ * a_;
    g_ = env.gravity;
    const double inv = (1.0 - ball.poissonRatio * ball.poissonRatio) / ball.youngsModulus +
                       (1.0 - plate.poissonRatio * plate.poissonRatio) / plate.youngsModulus;
    eStar_ = 1.0 / inv;
    dragK_ = 0.5 * env.airDensity * env.dragCoefficient * kPiD * a_ * a_;
}

double BallModel::hertzRadius(double N) const {
    if (N <= 0.0) return 0.0;
    return std::cbrt(3.0 * N * a_ / (4.0 * eStar_));
}

double BallModel::pivotingCouple(double N) const {
    return 3.0 * kPiD / 16.0 * c_.kineticFriction * N * hertzRadius(N);
}

Vec3d BallModel::gravityAndDrag(const Vec3d& v) const {
    return Vec3d{0.0, 0.0, -g_} - v * (dragK_ * v.norm() / m_);
}

Vec3d BallModel::resistingCouple(const BallState& s, const PlateMotion& pm, double N, const Vec3d& wRel) const {
    const Vec3d n = pm.pose.normal();
    // Rolling resistance: couple c_rr a N opposing the relative rolling, regularised linearly
    // inside the stick window (the Stuck mode handles true static resistance).
    const Vec3d wt = tangentialPart(wRel, n);
    const double wtMag = std::max(wt.norm(), c_.stickSpeed / a_);
    Vec3d M = wt * (-c_.rollingResistance * a_ * N / wtMag);
    // Pivoting (drilling) friction about the normal.
    const double Mmax = pivotingCouple(N);
    double Mn;
    if (s.spinStuck) {
        Mn = I_ * (pm.alpha.dot(n) - s.w.dot(pm.w.cross(n)));
        Mn = std::clamp(Mn, -Mmax, Mmax);
    } else {
        const double sr = wRel.dot(n);
        Mn = sr > 0.0 ? -Mmax : (sr < 0.0 ? Mmax : 0.0);
    }
    return M + n * Mn;
}

void BallModel::stuckWrench(const BallState& s, const PlateMotion& pm, Vec3d& F, Vec3d& M) const {
    const Vec3d n = pm.pose.normal();
    const Vec3d dv = pm.accelerationAt(s.r);
    F = (dv - gravityAndDrag(s.v)) * m_;
    M = pm.alpha * I_ + n.cross(F) * a_;
}

BallRates BallModel::rates(const BallState& s, const PlateMotion& pm) const {
    BallRates out;
    const Vec3d ge = gravityAndDrag(s.v);
    const Vec3d n = pm.pose.normal();
    switch (s.mode) {
        case ContactMode::Flight:
        case ContactMode::Lost:
            out.dv = ge;
            return out;

        case ContactMode::Stuck: {
            out.dv = pm.accelerationAt(s.r);
            out.force = (out.dv - ge) * m_;
            out.normalForce = out.force.dot(n);
            out.dw = pm.alpha;
            if (!s.spinStuck) {
                // relative spin decays under the sliding pivoting couple
                const double Mmax = pivotingCouple(std::max(out.normalForce, 0.0));
                const double sr = (s.w - pm.w).dot(n);
                const double Mn = sr > 0.0 ? -Mmax : (sr < 0.0 ? Mmax : 0.0);
                out.dw += n * (Mn / I_ - pm.alpha.dot(n));
            }
            out.moment = out.dw * I_ + n.cross(out.force) * a_;
            return out;
        }

        case ContactMode::Rolling: {
            const Vec3d c = s.r - n * a_;
            const Vec3d ndot = pm.w.cross(n);
            const Vec3d cdot = s.v - ndot * a_;
            // Second time derivative of the plate velocity at the moving contact point plus the
            // term from the rotating normal: b = d/dt[u(c)] + a w_b x n_dot.
            const Vec3d b = pm.accelerationAt(c) + pm.w.cross(cdot - pm.velocityAt(c)) + s.w.cross(ndot) * a_;
            const double bn = b.dot(n);
            const double N = m_ * (bn - ge.dot(n));
            const Vec3d M = resistingCouple(s, pm, std::max(N, 0.0), s.w - pm.w);
            const Vec3d bt = tangentialPart(b, n);
            const Vec3d gt = tangentialPart(ge, n);
            const Vec3d dvt = (bt * k_ + gt + M.cross(n) / (m_ * a_)) / (1.0 + k_);
            out.dv = dvt + n * bn;
            out.force = (out.dv - ge) * m_;
            out.moment = M;
            out.dw = (n.cross(out.force) * -a_ + M) / I_;
            out.normalForce = N;
            return out;
        }

        case ContactMode::Slipping: {
            const Vec3d c = s.r - n * a_;
            const Vec3d ndot = pm.w.cross(n);
            const Vec3d rp = s.r - pm.pose.p;
            const double bn = pm.a.dot(n) - 2.0 * (s.v - pm.v).dot(ndot) -
                              rp.dot(pm.alpha.cross(n) + pm.w.cross(ndot));
            const double N = m_ * (bn - ge.dot(n));
            const Vec3d slip = tangentialPart(s.v - s.w.cross(n) * a_ - pm.velocityAt(c), n);
            const double sm = slip.norm();
            Vec3d F = n * N;
            if (sm > 0.0) F += slip * (-c_.kineticFriction * std::max(N, 0.0) / sm);
            const Vec3d M = resistingCouple(s, pm, std::max(N, 0.0), s.w - pm.w);
            out.dv = ge + F / m_;
            out.dw = (n.cross(F) * -a_ + M) / I_;
            out.force = F;
            out.moment = M;
            out.normalForce = N;
            return out;
        }
    }
    return out;
}

void BallModel::projectContact(BallState& s, const PlateMotion& pm) const {
    const Vec3d n = pm.pose.normal();
    const double gap = (s.r - pm.pose.p).dot(n) - a_;
    s.r -= n * gap;
    const Vec3d c = s.r - n * a_;
    s.v -= n * (s.v - pm.velocityAt(c)).dot(n);
}

void BallModel::projectNoSlip(BallState& s, const PlateMotion& pm) const {
    // Remove the residual slip with a tangential impulse at the contact point,
    // P = -m k/(1+k) * slip. This conserves the ball's angular momentum about the contact
    // point, so correcting a slip overshoot does not change the rolling speed.
    const Vec3d n = pm.pose.normal();
    const Vec3d c = s.r - n * a_;
    const Vec3d slip = tangentialPart(s.v - s.w.cross(n) * a_ - pm.velocityAt(c), n);
    const Vec3d P = slip * (-m_ * k_ / (1.0 + k_));
    s.v += P / m_;
    s.w += n.cross(P) * (-a_ / I_);
}

BallState BallModel::restingOn(const PlateMotion& pm, double x, double y) const {
    BallState s;
    s.r = pm.pose.toWorld(Vec3d{x, y, a_});
    s.v = pm.velocityAt(s.r);
    s.w = pm.w;
    s.mode = ContactMode::Rolling;
    s.spinStuck = true;
    return s;
}

double BallModel::energy(const BallState& s) const {
    return 0.5 * m_ * s.v.squaredNorm() + 0.5 * I_ * s.w.squaredNorm() + m_ * g_ * s.r.z;
}

void BallModel::postStep(BallState& s, const PlateMotion& pm, BallEvents* ev) const {
    BallEvents local;
    BallEvents& e = ev ? *ev : local;
    const Vec3d n = pm.pose.normal();
    const Vec3d xP = pm.pose.toPlate(s.r);
    const double radial = std::hypot(xP.x, xP.y);
    const double gap = xP.z - a_;

    if (s.mode == ContactMode::Lost) return;

    if (s.mode == ContactMode::Flight) {
        if (radial > plateRadius_ && xP.z < 0.0) {
            s.mode = ContactMode::Lost;
            e.fellOff = true;
            return;
        }
        if (radial <= plateRadius_ && gap <= 0.0) {
            s.r -= n * gap;
            const Vec3d c = s.r - n * a_;
            const double vn = (s.v - pm.velocityAt(c)).dot(n);
            if (vn < 0.0) {
                // Newton impact with a Coulomb-limited tangential impulse (Stronge 2000).
                const double eN = (-vn < c_.restingSpeed) ? 0.0 : c_.restitution;
                const double Pn = -m_ * (1.0 + eN) * vn;
                const Vec3d slip = tangentialPart(s.v - s.w.cross(n) * a_ - pm.velocityAt(c), n);
                Vec3d Pt = slip * (-m_ * k_ / (1.0 + k_));
                const double PtMax = c_.kineticFriction * Pn;
                if (Pt.norm() > PtMax) Pt = Pt * (PtMax / Pt.norm());
                s.v += (n * Pn + Pt) / m_;
                s.w += n.cross(Pt) * (-a_ / I_);
                e.impact = true;
                e.impactSpeed = -vn;
                if (eN * -vn < c_.restingSpeed) {
                    const Vec3d slip2 = tangentialPart(s.v - s.w.cross(n) * a_ - pm.velocityAt(c), n);
                    if (slip2.norm() < 10.0 * c_.slipSpeed) {
                        s.mode = ContactMode::Rolling;
                        projectContact(s, pm);
                        projectNoSlip(s, pm);
                    } else {
                        s.mode = ContactMode::Slipping;
                        s.slipDir = slip2.normalized();
                        projectContact(s, pm);
                    }
                    s.spinStuck = false;
                    s.spinSign = 0.0;
                }
            }
        }
        return;
    }

    // Contact modes.
    if (radial > plateRadius_) {
        s.mode = ContactMode::Flight;
        e.fellOff = true;
        return;
    }
    projectContact(s, pm);
    BallRates rt = rates(s, pm);
    if (rt.normalForce < 0.0) {
        s.mode = ContactMode::Flight;
        e.separated = true;
        return;
    }
    const double N = rt.normalForce;
    const double muS = c_.staticFriction;

    switch (s.mode) {
        case ContactMode::Rolling: {
            const Vec3d Ft = tangentialPart(rt.force, n);
            if (Ft.norm() > muS * N) {
                const Vec3d c = s.r - n * a_;
                const Vec3d slip = tangentialPart(s.v - s.w.cross(n) * a_ - pm.velocityAt(c), n);
                s.mode = ContactMode::Slipping;
                s.slipDir = slip.norm() > 0.0 ? slip.normalized() : Ft.normalized() * -1.0;
                break;
            }
            projectNoSlip(s, pm);
            const Vec3d wt = tangentialPart(s.w - pm.w, n);
            if (wt.norm() * a_ < c_.stickSpeed) {
                Vec3d F, M;
                stuckWrench(s, pm, F, M);
                if (tangentialPart(F, n).norm() <= muS * N &&
                    tangentialPart(M, n).norm() <= c_.rollingResistance * a_ * N) {
                    s.mode = ContactMode::Stuck;
                    s.v = pm.velocityAt(s.r);
                    s.w = tangentialPart(pm.w, n) + n * s.w.dot(n);
                }
            }
            break;
        }
        case ContactMode::Stuck: {
            Vec3d F, M;
            stuckWrench(s, pm, F, M);
            if (tangentialPart(F, n).norm() > muS * N) {
                s.mode = ContactMode::Slipping;
                s.slipDir = tangentialPart(F, n).normalized() * -1.0;
            } else if (tangentialPart(M, n).norm() > c_.rollingResistance * a_ * N) {
                s.mode = ContactMode::Rolling;
                projectNoSlip(s, pm);
            } else {
                s.v = pm.velocityAt(s.r);
                s.w = tangentialPart(pm.w, n) + n * s.w.dot(n);
            }
            break;
        }
        case ContactMode::Slipping: {
            const Vec3d c = s.r - n * a_;
            const Vec3d slip = tangentialPart(s.v - s.w.cross(n) * a_ - pm.velocityAt(c), n);
            if (slip.norm() < c_.slipSpeed || slip.dot(s.slipDir) <= 0.0) {
                s.mode = ContactMode::Rolling;
                projectNoSlip(s, pm);
            } else {
                s.slipDir = slip.normalized();
            }
            break;
        }
        default:
            break;
    }

    // Pivoting friction about the contact normal (applies to every contact mode).
    const double wRelN = (s.w - pm.w).dot(n);
    const double Mmax = pivotingCouple(N);
    if (s.spinStuck) {
        const double Mreq = I_ * (pm.alpha.dot(n) - s.w.dot(pm.w.cross(n)));
        if (std::fabs(Mreq) > Mmax) {
            s.spinStuck = false;
            s.spinSign = wRelN != 0.0 ? (wRelN > 0.0 ? 1.0 : -1.0) : (Mreq > 0.0 ? -1.0 : 1.0);
        } else {
            s.w -= n * wRelN;
        }
    } else if (std::fabs(wRelN) < c_.spinStickRate || wRelN * s.spinSign < 0.0) {
        s.spinStuck = true;
        s.w -= n * wRelN;
    } else {
        s.spinSign = wRelN > 0.0 ? 1.0 : -1.0;
    }
}

void BallIntegrator::step(BallState& s, double& t, BallEvents* ev) const {
    const double h = dt_;
    const PlateMotion m0 = motion_(t), m1 = motion_(t + 0.5 * h), m2 = motion_(t + h);
    auto f = [&](const BallState& x, const PlateMotion& pm, Vec3d& dr, Vec3d& dv, Vec3d& dw) {
        const BallRates r = model_.rates(x, pm);
        dr = x.v;
        dv = r.dv;
        dw = r.dw;
    };
    Vec3d r1, v1, w1, r2, v2, w2, r3, v3, w3, r4, v4, w4;
    f(s, m0, r1, v1, w1);
    BallState x = s;
    x.r = s.r + r1 * (0.5 * h); x.v = s.v + v1 * (0.5 * h); x.w = s.w + w1 * (0.5 * h);
    f(x, m1, r2, v2, w2);
    x.r = s.r + r2 * (0.5 * h); x.v = s.v + v2 * (0.5 * h); x.w = s.w + w2 * (0.5 * h);
    f(x, m1, r3, v3, w3);
    x.r = s.r + r3 * h; x.v = s.v + v3 * h; x.w = s.w + w3 * h;
    f(x, m2, r4, v4, w4);
    s.r += (r1 + r2 * 2.0 + r3 * 2.0 + r4) * (h / 6.0);
    s.v += (v1 + v2 * 2.0 + v3 * 2.0 + v4) * (h / 6.0);
    s.w += (w1 + w2 * 2.0 + w3 * 2.0 + w4) * (h / 6.0);
    t += h;
    model_.postStep(s, m2, ev);
}

}  // namespace bbp::sim
