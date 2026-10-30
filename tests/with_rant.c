/* Rant Math's implementation beside Rant in one file, Rant first when RANT_FIRST is set.
 * It only has to compile: the types both carry are defined once. */
#ifdef RANT_FIRST
#include "rant.h"
#endif
#define RANT_MATH_IMPLEMENTATION
#include "rant_math.h"
#ifndef RANT_FIRST
#include "rant.h"
#endif

RantPose with_rant_pose(RantPose p){ return rant_pose_inverse(p); }
