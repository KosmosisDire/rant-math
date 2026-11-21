/* Both libraries built in one file, in the order the build says. It only has to compile. */
#define RANT_IMPLEMENTATION
#define RANT_MATH_IMPLEMENTATION
#if defined(RANT_FIRST)
#include "rant.hpp"
#include "rant_math.hpp"
#else
#include "rant_math.hpp"
#include "rant.hpp"
#endif
