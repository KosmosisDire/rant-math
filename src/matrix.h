/* Matrices: the f32 wire types Matrix3x3 and Matrix4x4 computed in double, and any other as a
 * RantMatrixView. Points are columns, p' = M p, so the translation is m[0][3], m[1][3], m[2][3]. */
#ifndef RANT_MATH_MATRIX_H
#define RANT_MATH_MATRIX_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

RANT_MATH_API RantMatrix3x3  rant_matrix3x3_identity(void);
RANT_MATH_API RantMatrix3x3  rant_matrix3x3_multiply(RantMatrix3x3 a, RantMatrix3x3 b);
RANT_MATH_API RantMatrix3x3  rant_matrix3x3_transpose(RantMatrix3x3 m);
RANT_MATH_API double         rant_matrix3x3_determinant(RantMatrix3x3 m);
RANT_MATH_API RantMathResult rant_matrix3x3_inverse(RantMatrix3x3 m, RantMatrix3x3 *out);
RANT_MATH_API RantDouble3    rant_matrix3x3_apply(RantMatrix3x3 m, RantDouble3 v);

RANT_MATH_API RantMatrix4x4  rant_matrix4x4_identity(void);
RANT_MATH_API RantMatrix4x4  rant_matrix4x4_multiply(RantMatrix4x4 a, RantMatrix4x4 b);
RANT_MATH_API RantMatrix4x4  rant_matrix4x4_transpose(RantMatrix4x4 m);
RANT_MATH_API double         rant_matrix4x4_determinant(RantMatrix4x4 m);
RANT_MATH_API RantMathResult rant_matrix4x4_inverse(RantMatrix4x4 m, RantMatrix4x4 *out);
/* A point, divided through by w when the bottom row is not 0 0 0 1. */
RANT_MATH_API RantDouble3    rant_matrix4x4_apply(RantMatrix4x4 m, RantDouble3 point);
/* A direction: the 3 by 3 part only, with no translation. */
RANT_MATH_API RantDouble3    rant_matrix4x4_apply_direction(RantMatrix4x4 m, RantDouble3 direction);

/* A dense matrix of doubles, rows one after another. A Covariance6 is { &c.m[0][0], 6, 6 }.
 * An output never shares memory with an input, and a call refuses one that does. */
typedef struct { double *data; uint32_t rows, cols; } RantMatrixView;

RANT_MATH_API RantMathResult rant_matrix_multiply(RantMatrixView a, RantMatrixView b, RantMatrixView out);
RANT_MATH_API RantMathResult rant_matrix_transpose(RantMatrixView a, RantMatrixView out);
RANT_MATH_API RantMathResult rant_matrix_determinant(RantMatrixView a, double *out);
RANT_MATH_API RantMathResult rant_matrix_inverse(RantMatrixView a, RantMatrixView out);
/* x such that a x = b, each column of b solved alone. With more rows than columns, the x
 * with the least squared error. Fails when the columns of a are dependent. */
RANT_MATH_API RantMathResult rant_matrix_solve(RantMatrixView a, RantMatrixView b, RantMatrixView x);
/* a = u diag(s) v^T for an m by n matrix and k = min(m, n): u is m by k, s holds k values
 * from the largest down, v is n by k. */
RANT_MATH_API RantMathResult rant_matrix_svd(RantMatrixView a, RantMatrixView u, double *s, RantMatrixView v);
/* A symmetric matrix's eigenvalues from the smallest up, each eigenvector a column. */
RANT_MATH_API RantMathResult rant_matrix_eigen(RantMatrixView a, double *values, RantMatrixView vectors);
/* The lower l with a = l l^T. Fails when a is not symmetric positive definite. */
RANT_MATH_API RantMathResult rant_matrix_cholesky(RantMatrixView a, RantMatrixView l);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_MATRIX_H */
