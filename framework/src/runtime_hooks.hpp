// Connection between the student library (bbp.hpp) and the runtime that hosts it.
#pragma once

#include <functional>

namespace bbp::detail {

using LogSink = std::function<void(const char* name, float value)>;

void setLogSink(LogSink sink);
void logValue(const char* name, float value);

}  // namespace bbp::detail
