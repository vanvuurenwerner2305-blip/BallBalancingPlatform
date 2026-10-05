// Minimal test harness (no external dependencies). Each TEST registers itself; bbp_tests runs
// them all, or those whose name contains the first command-line argument. With
// --csv <dir> the tests also write the data series used for the figures in docs/physics.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

namespace bbptest {

struct Case {
    const char* name;
    std::function<void()> fn;
};

std::vector<Case>& registry();
extern int failures;
extern std::string csvDir;

struct Registrar {
    Registrar(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};

inline void check(bool ok, const char* expr, const char* file, int line) {
    if (!ok) {
        std::printf("    FAILED %s:%d: %s\n", file, line, expr);
        ++failures;
    }
}

inline void checkNear(double a, double b, double tol, const char* ea, const char* eb, const char* file, int line) {
    if (!(std::fabs(a - b) <= tol)) {
        std::printf("    FAILED %s:%d: |%s - %s| = |%.9g - %.9g| = %.3g > %.3g\n", file, line, ea, eb, a, b,
                    std::fabs(a - b), tol);
        ++failures;
    }
}

// Prints a measured quantity next to its expected value (recorded in the paper).
inline void report(const char* what, double measured, double expected) {
    std::printf("    %-46s measured %.7g  expected %.7g  rel.err %.2e\n", what, measured, expected,
                expected != 0.0 ? std::fabs(measured - expected) / std::fabs(expected) : std::fabs(measured));
}

// CSV output, active only when --csv was given.
class Csv {
public:
    Csv(const std::string& name, const std::string& header) {
        if (csvDir.empty()) return;
        f_.open(csvDir + "/" + name);
        f_ << header << "\n";
    }
    template <typename... Ts>
    void row(Ts... vals) {
        if (!f_.is_open()) return;
        bool first = true;
        ((f_ << (first ? "" : ",") << vals, first = false), ...);
        f_ << "\n";
    }
    bool active() const { return f_.is_open(); }

private:
    std::ofstream f_;
};

}  // namespace bbptest

#define BBP_CAT2(a, b) a##b
#define BBP_CAT(a, b) BBP_CAT2(a, b)
#define TEST(name)                                                         \
    static void name();                                                    \
    static bbptest::Registrar BBP_CAT(reg_, name)(#name, name);            \
    static void name()
#define CHECK(c) bbptest::check((c), #c, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) bbptest::checkNear((a), (b), (tol), #a, #b, __FILE__, __LINE__)
