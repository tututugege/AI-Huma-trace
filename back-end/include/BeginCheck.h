#pragma once
// Working copies (X_1) equal the committed state (X) at the start of every cycle because seq() copies them back.
// A build with -DBEGIN_SELFCHECK verifies that at each comb_begin instead of trusting it.
#ifdef BEGIN_SELFCHECK
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define BEGIN_EQ(a, b)                                                                          \
  do {                                                                                          \
    static_assert(sizeof(a) == sizeof(b), "BEGIN_EQ size");                                     \
    if (std::memcmp(&(a), &(b), sizeof(a)) != 0) {                                              \
      std::fprintf(stderr, "[BEGIN_SELFCHECK] %s != %s (%s:%d)\n", #a, #b, __FILE__, __LINE__); \
      std::abort();                                                                             \
    }                                                                                           \
  } while (0)
#else
#define BEGIN_EQ(a, b) ((void)0)
#endif
