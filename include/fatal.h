#pragma once
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

// A broken model invariant (or an impossible input): print the message (printf format) and stop the simulation.
[[noreturn]] inline void fatal(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  std::vprintf(fmt, ap);
  va_end(ap);
  std::fflush(stdout);
  std::exit(1);
}
