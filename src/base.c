#include "internal.h"

const char *rant_math_result_text(RantMathResult r){
    switch (r){
    case RANT_MATH_OK:             return "ok";
    case RANT_MATH_ERR_ARGUMENT:   return "a NULL pointer, a count too small, or a value out of range";
    case RANT_MATH_ERR_DEGENERATE: return "no single answer: a singular matrix or points on one line";
    case RANT_MATH_ERR_SPACE:      return "the output is too small";
    case RANT_MATH_ERR_FORMAT:     return "a layout this call cannot read";
    case RANT_MATH_ERR_MEMORY:     return "out of memory";
    case RANT_MATH_ERR_OUTSIDE:    return "a point the call cannot map";
    }
    return "an unknown result";
}
