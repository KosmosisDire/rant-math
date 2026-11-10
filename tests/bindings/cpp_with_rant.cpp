/* rant_math.hpp and rant.hpp in one file, in the order the build says, then alone. It only has to
 * compile: one Pose in both, and values pass between them with no conversion. */
#if defined(RANT_FIRST)
#include "rant.hpp"
#include "rant_math.hpp"
#elif defined(MATH_FIRST)
#include "rant_math.hpp"
#include "rant.hpp"
#elif defined(TYPES_FIRST)
#include "rant_types.hpp"
#include "rant_math.hpp"
#else
#include "rant_math.hpp"
#endif

rant::types::Pose with_rant_inverse(rant::types::Pose p) {
    return rant::math::pose_inverse(p);
}
