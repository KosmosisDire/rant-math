#include <stdlib.h>
#include "check.h"
#include "reference.h"

static RantMatrixView view(double *d, uint32_t r, uint32_t c){
    RantMatrixView v;
    v.data = d; v.rows = r; v.cols = c;
    return v;
}

static int near_all(const double *a, const double *b, size_t n, double tol){
    size_t i;
    for (i = 0; i < n; i++) if (!near(a[i], b[i], tol)) return 0;
    return 1;
}

/* |a^T a - I| for the columns of the r by c a. */
static double orthonormal_error(const double *a, uint32_t r, uint32_t c){
    double worst = 0.0;
    uint32_t i, j, k;
    for (i = 0; i < c; i++)
        for (j = 0; j < c; j++){
            double s = 0.0;
            for (k = 0; k < r; k++) s += a[k * c + i] * a[k * c + j];
            s -= i == j ? 1.0 : 0.0;
            if (fabs(s) > worst) worst = fabs(s);
        }
    return worst;
}

static void check_svd(const double *a, uint32_t m, uint32_t n, const double *want_s, const char *what){
    uint32_t k = m < n ? m : n, i, j, p;
    double *u = (double *)malloc(sizeof(double) * m * k), *v = (double *)malloc(sizeof(double) * n * k);
    double s[16], worst = 0.0;
    RantMathResult r = rant_matrix_svd(view((double *)a, m, n), view(u, m, k), s, view(v, n, k));
    CHECK(r == RANT_MATH_OK, "%s svd runs", what);
    if (want_s) CHECK(near_all(s, want_s, k, 1e-12), "%s singular values match numpy", what);
    for (i = 0; i + 1 < k; i++) CHECK(s[i] >= s[i + 1], "%s singular values descend", what);
    for (i = 0; i < m; i++)
        for (j = 0; j < n; j++){
            double sum = 0.0;
            for (p = 0; p < k; p++) sum += u[i * k + p] * s[p] * v[j * k + p];
            if (fabs(sum - a[i * n + j]) > worst) worst = fabs(sum - a[i * n + j]);
        }
    CHECK(worst < 1e-12, "%s u s v^T rebuilds a (%g)", what, worst);
    CHECK(orthonormal_error(u, m, k) < 1e-12 && orthonormal_error(v, n, k) < 1e-12, "%s u and v are orthonormal", what);
    free(u); free(v);
}

static void fixed(void){
    RantMatrix3x3 m3 = rant_matrix3x3_identity(), inv3;
    RantMatrix4x4 m4 = rant_matrix4x4_identity(), inv4, prod;
    int i, j, ok = 1;
    m3.m[0][1] = 2.0f; m3.m[2][0] = -1.0f; m3.m[1][1] = 3.0f;
    CHECK(near(rant_matrix3x3_determinant(m3), 3.0, 1e-12), "3x3 determinant");
    CHECK(rant_matrix3x3_inverse(m3, &inv3) == RANT_MATH_OK, "3x3 inverse runs");
    {   RantMatrix3x3 p = rant_matrix3x3_multiply(m3, inv3);
        for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) if (!near(p.m[i][j], i == j, 1e-6)) ok = 0;
        CHECK(ok, "3x3 times its inverse is the identity");
    }
    CHECK(near3(rant_matrix3x3_apply(m3, rant_double3(1, 1, 1)), rant_double3(3, 3, 0), 1e-12), "3x3 apply");
    {   RantMatrix3x3 t = rant_matrix3x3_transpose(m3);
        CHECK(t.m[1][0] == 2.0f, "3x3 transpose");
    }
    {   RantMatrix3x3 s = rant_matrix3x3_identity(), dummy = rant_matrix3x3_identity();
        s.m[2][2] = 0.0f;
        CHECK(rant_matrix3x3_inverse(s, &dummy) == RANT_MATH_ERR_DEGENERATE && dummy.m[2][2] == 1.0f,
              "a singular 3x3 refuses and leaves out alone");
        CHECK(rant_matrix3x3_inverse(s, NULL) == RANT_MATH_ERR_ARGUMENT, "3x3 inverse wants an out");
    }
    m4.m[0][3] = 5.0f; m4.m[1][2] = -2.0f; m4.m[3][0] = 0.5f;
    CHECK(rant_matrix4x4_inverse(m4, &inv4) == RANT_MATH_OK, "4x4 inverse runs");
    prod = rant_matrix4x4_multiply(inv4, m4);
    ok = 1;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) if (!near(prod.m[i][j], i == j, 1e-6)) ok = 0;
    CHECK(ok, "4x4 inverse times it is the identity");
    CHECK(near(rant_matrix4x4_determinant(m4), -1.5, 1e-6), "4x4 determinant");
    {   RantMatrix4x4 t = rant_matrix4x4_identity();
        t.m[0][3] = 1.0f; t.m[1][3] = 2.0f; t.m[2][3] = 3.0f;
        CHECK(near3(rant_matrix4x4_apply(t, rant_double3(1, 1, 1)), rant_double3(2, 3, 4), 0.0),
              "4x4 apply moves a point by its last column");
        CHECK(near3(rant_matrix4x4_apply_direction(t, rant_double3(1, 1, 1)), rant_double3(1, 1, 1), 0.0),
              "4x4 apply_direction ignores the translation");
        t.m[3][3] = 2.0f;
        CHECK(near3(rant_matrix4x4_apply(t, rant_double3(1, 1, 1)), rant_double3(1, 1.5, 2), 1e-15),
              "4x4 apply divides by w");
        t = rant_matrix4x4_transpose(t);
        CHECK(t.m[3][0] == 1.0f, "4x4 transpose");
    }
}

static void dense(void){
    double inv[25], det, x[6], l[36], dummy[36];
    RantMathResult r;
    check_svd(REF_SVD_A, 7, 4, REF_SVD_S, "tall");
    check_svd(REF_SVD_WIDE_A, 3, 6, REF_SVD_WIDE_S, "wide");
    {   /* rank 2: the third column is the sum of the first two, so u must be completed */
        double a[12] = { 1, 2, 3,  4, 5, 9,  -1, 0, -1,  2, 2, 4 };
        double s[3], u[12], v[9];
        check_svd(a, 4, 3, NULL, "rank deficient");
        rant_matrix_svd(view(a, 4, 3), view(u, 4, 3), s, view(v, 3, 3));
        CHECK(s[2] < 1e-12, "rank deficient: the last singular value is zero (%g)", s[2]);
    }
    {   double a[1] = { -3.0 }, u[1], v[1], s[1];
        CHECK(rant_matrix_svd(view(a, 1, 1), view(u, 1, 1), s, view(v, 1, 1)) == RANT_MATH_OK && s[0] == 3.0
              && u[0] * v[0] == -1.0, "svd of a 1 by 1");
    }
    {   double values[6], vectors[36], av[36];
        uint32_t i, j;
        r = rant_matrix_eigen(view((double *)REF_EIGEN_A, 6, 6), values, view(vectors, 6, 6));
        CHECK(r == RANT_MATH_OK && near_all(values, REF_EIGEN_VALUES, 6, 1e-12), "eigenvalues match numpy");
        rant_matrix_multiply(view((double *)REF_EIGEN_A, 6, 6), view(vectors, 6, 6), view(av, 6, 6));
        for (j = 0; j < 6; j++){
            int ok = 1;
            for (i = 0; i < 6; i++) if (!near(av[i * 6 + j], values[j] * vectors[i * 6 + j], 1e-11)) ok = 0;
            CHECK(ok, "eigenvector %u satisfies a v = l v", j);
        }
        CHECK(orthonormal_error(vectors, 6, 6) < 1e-12, "eigenvectors are orthonormal");
        {   double asym[4] = { 1, 2, 0, 1 }, vv[4], vals[2];
            CHECK(rant_matrix_eigen(view(asym, 2, 2), vals, view(vv, 2, 2)) == RANT_MATH_ERR_ARGUMENT,
                  "eigen refuses a matrix that is not symmetric");
        }
    }
    r = rant_matrix_inverse(view((double *)REF_INVERSE_A, 5, 5), view(inv, 5, 5));
    CHECK(r == RANT_MATH_OK && near_all(inv, REF_INVERSE, 25, 1e-12), "inverse matches numpy");
    r = rant_matrix_determinant(view((double *)REF_INVERSE_A, 5, 5), &det);
    CHECK(r == RANT_MATH_OK && near(det, REF_DETERMINANT, 1e-12), "determinant matches numpy");
    {   double sing[9] = { 1, 2, 3, 2, 4, 6, 0, 1, 1 }, out[9] = { 7 };
        CHECK(rant_matrix_inverse(view(sing, 3, 3), view(out, 3, 3)) == RANT_MATH_ERR_DEGENERATE && out[0] == 7,
              "a singular inverse refuses and leaves out alone");
        CHECK(rant_matrix_determinant(view(sing, 3, 3), &det) == RANT_MATH_OK && near(det, 0.0, 1e-12),
              "a singular determinant is 0");
        CHECK(rant_matrix_inverse(view(sing, 3, 3), view(sing, 3, 3)) == RANT_MATH_ERR_ARGUMENT,
              "an output sharing memory with its input is refused");
    }
    r = rant_matrix_solve(view((double *)REF_LSQ_A, 9, 3), view((double *)REF_LSQ_B, 9, 2), view(x, 3, 2));
    CHECK(r == RANT_MATH_OK && near_all(x, REF_LSQ_X, 6, 1e-12), "least squares matches numpy");
    {   double a[4] = { 2, 1, 1, 3 }, b[2] = { 3, 5 }, s[2];
        CHECK(rant_matrix_solve(view(a, 2, 2), view(b, 2, 1), view(s, 2, 1)) == RANT_MATH_OK
              && near(s[0], 0.8, 1e-15) && near(s[1], 1.4, 1e-15), "square solve");
    }
    {   double a[6] = { 1, 2, 2, 4, 3, 6 }, b[3] = { 1, 2, 3 }, s[2] = { 9, 9 };
        CHECK(rant_matrix_solve(view(a, 3, 2), view(b, 3, 1), view(s, 2, 1)) == RANT_MATH_ERR_DEGENERATE && s[0] == 9,
              "least squares refuses dependent columns");
        CHECK(rant_matrix_solve(view(a, 2, 3), view(b, 2, 1), view(s, 3, 1)) == RANT_MATH_ERR_ARGUMENT,
              "solve refuses fewer rows than columns");
    }
    r = rant_matrix_cholesky(view((double *)REF_CHOLESKY_A, 6, 6), view(l, 6, 6));
    CHECK(r == RANT_MATH_OK && near_all(l, REF_CHOLESKY_L, 36, 1e-12), "cholesky matches numpy");
    {   double a[4] = { 1, 2, 2, 1 };
        dummy[0] = 5.0;
        CHECK(rant_matrix_cholesky(view(a, 2, 2), view(dummy, 2, 2)) == RANT_MATH_ERR_DEGENERATE && dummy[0] == 5.0,
              "cholesky refuses an indefinite matrix and leaves l alone");
    }
    {   double a[6] = { 1, 2, 3, 4, 5, 6 }, t[6], p[4];
        CHECK(rant_matrix_transpose(view(a, 2, 3), view(t, 3, 2)) == RANT_MATH_OK && t[1] == 4 && t[4] == 3, "transpose");
        CHECK(rant_matrix_multiply(view(a, 2, 3), view(t, 3, 2), view(p, 2, 2)) == RANT_MATH_OK
              && p[0] == 14 && p[1] == 32 && p[3] == 77, "multiply");
        CHECK(rant_matrix_multiply(view(a, 2, 3), view(a, 2, 3), view(p, 2, 2)) == RANT_MATH_ERR_ARGUMENT,
              "multiply refuses mismatched sizes");
        CHECK(rant_matrix_multiply(view(NULL, 2, 3), view(t, 3, 2), view(p, 2, 2)) == RANT_MATH_ERR_ARGUMENT,
              "multiply refuses NULL");
    }
    {   RantCovariance6 c;
        double values[6], vectors[36];
        int i, j;
        for (i = 0; i < 6; i++) for (j = 0; j < 6; j++) c.m[i][j] = i == j ? (double)(i + 1) : 0.0;
        CHECK(rant_matrix_eigen(view(&c.m[0][0], 6, 6), values, view(vectors, 6, 6)) == RANT_MATH_OK
              && values[0] == 1.0 && values[5] == 6.0, "a Covariance6 is a 6 by 6 view");
    }
}

void test_matrix(void){
    fixed();
    dense();
}
