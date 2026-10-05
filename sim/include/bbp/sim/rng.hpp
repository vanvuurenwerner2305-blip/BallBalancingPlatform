// Deterministic random numbers. The C++ standard library distributions are implementation
// defined (MSVC and libstdc++ give different normal variates for the same seed), so the
// simulator uses its own generator to make runs bit-reproducible across compilers and
// independent of thread scheduling.
#pragma once

#include <cmath>
#include <cstdint>

namespace bbp::sim {

// SplitMix64 finaliser: a high-quality 64-bit mixing function.
inline std::uint64_t mix64(std::uint64_t z) {
    z += 0x9e3779b97f4a7c15ULL;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

// Counter-based stream: the n-th number depends only on (seed, stream, n), so pixels can be
// rendered in any order or in parallel and still get identical noise.
class Rng {
public:
    explicit Rng(std::uint64_t seed = 0, std::uint64_t stream = 0) : key_(mix64(seed ^ mix64(stream))) {}

    std::uint64_t next() { return mix64(key_ + 0x632be59bd9b4e019ULL * counter_++); }

    // Uniform in (0, 1), never exactly 0 or 1.
    double uniform() { return (double(next() >> 11) + 0.5) * (1.0 / 9007199254740992.0); }

    // Standard normal via Box-Muller.
    double normal() {
        if (hasSpare_) {
            hasSpare_ = false;
            return spare_;
        }
        const double u1 = uniform(), u2 = uniform();
        const double r = std::sqrt(-2.0 * std::log(u1));
        const double a = 6.283185307179586 * u2;
        spare_ = r * std::sin(a);
        hasSpare_ = true;
        return r * std::cos(a);
    }

private:
    std::uint64_t key_;
    std::uint64_t counter_ = 0;
    double spare_ = 0.0;
    bool hasSpare_ = false;
};

}  // namespace bbp::sim
