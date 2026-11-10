/* The library describing itself: every call as the bindings see it, and the methods and operators each
 * type has, for a tool or a binding that finds a call by its name. tools/bindgen writes table.c. */
#ifndef RANT_MATH_TABLE_H
#define RANT_MATH_TABLE_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One thing a call takes or gives. kind says how a binding passes it: value, ref, list, shape, image, out,
 * query and the rest of bindgen's words. type is a standard type's name, a C number type, or "". */
typedef struct {
    const char *name;
    const char *kind;
    const char *type;
} RantMathArg;

/* A call by its name without rant_, as pose_inverse. errors is none, throw, optional or bool: how a binding
 * answers a failure, a miss or a yes or no. */
typedef struct {
    const char *name;
    const char *doc;
    const RantMathArg *inputs;
    uint32_t input_count;
    const RantMathArg *outputs;
    uint32_t output_count;
    const char *errors;
} RantMathFunction;

/* A call as a method of type: self_index names the input that is the value, field the field of it the call
 * reads, as points of a Polygon. A static has no value and self_index UINT32_MAX. */
typedef struct {
    const char *type;
    const char *name;
    uint32_t function;
    uint32_t self_index;
    const char *field;
} RantMathMethod;

/* left op right, as Pose * Double3. op is + - * / or neg, how is call, scale, scale_left, divide or negate. */
typedef struct {
    const char *op;
    const char *left;
    const char *right;
    uint32_t function;
    const char *how;
} RantMathOperator;

RANT_MATH_API uint32_t                rant_math_function_count(void);
RANT_MATH_API const RantMathFunction *rant_math_function_at(uint32_t index);   /* NULL past the end */
RANT_MATH_API uint32_t                rant_math_method_count(void);
RANT_MATH_API const RantMathMethod   *rant_math_method_at(uint32_t index);
RANT_MATH_API uint32_t                rant_math_operator_count(void);
RANT_MATH_API const RantMathOperator *rant_math_operator_at(uint32_t index);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_TABLE_H */
