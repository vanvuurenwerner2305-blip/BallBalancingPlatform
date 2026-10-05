#include <cstring>

#include "test.hpp"

namespace bbptest {
std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}
int failures = 0;
std::string csvDir;
}  // namespace bbptest

int main(int argc, char** argv) {
    const char* filter = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
            bbptest::csvDir = argv[++i];
        } else {
            filter = argv[i];
        }
    }
    int run = 0, failedCases = 0;
    for (const auto& c : bbptest::registry()) {
        if (filter && !std::strstr(c.name, filter)) continue;
        std::printf("[ RUN  ] %s\n", c.name);
        const int before = bbptest::failures;
        c.fn();
        const bool ok = bbptest::failures == before;
        std::printf("[ %s ] %s\n", ok ? " OK " : "FAIL", c.name);
        ++run;
        if (!ok) ++failedCases;
    }
    std::printf("\n%d test(s) run, %d failed\n", run, failedCases);
    return failedCases == 0 ? 0 : 1;
}
