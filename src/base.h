/* What every part of the library shares: the linkage, the result codes and the views of
 * standard types that have no C mirror. */
#ifndef RANT_MATH_BASE_H
#define RANT_MATH_BASE_H

#include <stddef.h>
#include <stdint.h>
#include "rant_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Default: compiled into the caller's own binary. RANT_MATH_BUILD_SHARED builds the shared
 * library, RANT_MATH_LINK_SHARED consumes one. */
#if defined(RANT_MATH_BUILD_SHARED)
  #if defined(_WIN32)
    #define RANT_MATH_API __declspec(dllexport)
  #elif defined(__EMSCRIPTEN__)
    #define RANT_MATH_API __attribute__((used, visibility("default")))
  #else
    #define RANT_MATH_API __attribute__((visibility("default")))
  #endif
#elif defined(RANT_MATH_LINK_SHARED) && defined(_WIN32)
  #define RANT_MATH_API __declspec(dllimport)
#else
  #define RANT_MATH_API
#endif

/* 0 ok, negative on failure. A call that can fail returns one and writes its answer through
 * its last pointer, which it leaves untouched on failure. */
typedef enum {
    RANT_MATH_OK             =  0,
    RANT_MATH_ERR_ARGUMENT   = -1,  /* a NULL pointer, a count too small, or a value out of range */
    RANT_MATH_ERR_DEGENERATE = -2,  /* no single answer: a singular matrix, points on one line */
    RANT_MATH_ERR_SPACE      = -3,  /* the output is too small: the count it needs is written back */
    RANT_MATH_ERR_FORMAT     = -4,  /* an image or cloud laid out in a way this call cannot read */
    RANT_MATH_ERR_MEMORY     = -5,  /* the allocator returned NULL */
    RANT_MATH_ERR_OUTSIDE    = -6   /* a point the call cannot map, such as one behind the camera */
} RantMathResult;

/* The count rule of a call that writes a list: *count holds the room and gets back the need. */

/* A result as words, for an error message. */
RANT_MATH_API const char *rant_math_result_text(RantMathResult r);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_BASE_H */
