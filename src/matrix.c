#include "internal.h"

/* The dense engines. */

/* Each engine works in place on caller memory, so the fixed sizes run with no heap. */

static double i_max_abs(const double *a, size_t n){
    double m = 0.0;
    size_t i;
    for (i = 0; i < n; i++) if (fabs(a[i]) > m) m = fabs(a[i]);
    return m;
}

/* LU with partial pivoting of the n by n a, in place. perm records the row swaps. Returns
 * the sign of the permutation, or 0 when a pivot is exactly zero. */
static int i_lu(double *a, int n, int *perm){
    int i, j, k, sign = 1;
    for (i = 0; i < n; i++) perm[i] = i;
    for (k = 0; k < n; k++){
        int p = k;
        double best = fabs(a[k * n + k]);
        for (i = k + 1; i < n; i++)
            if (fabs(a[i * n + k]) > best){ best = fabs(a[i * n + k]); p = i; }
        if (best == 0.0) return 0;
        if (p != k){
            int t = perm[k]; perm[k] = perm[p]; perm[p] = t;
            for (j = 0; j < n; j++){ double s = a[k * n + j]; a[k * n + j] = a[p * n + j]; a[p * n + j] = s; }
            sign = -sign;
        }
        for (i = k + 1; i < n; i++){
            double f = a[i * n + k] / a[k * n + k];
            a[i * n + k] = f;
            for (j = k + 1; j < n; j++) a[i * n + j] -= f * a[k * n + j];
        }
    }
    return sign;
}

static double i_lu_determinant(const double *lu, int n, int sign){
    double d = (double)sign;
    int i;
    for (i = 0; i < n; i++) d *= lu[i * n + i];
    return d;
}

/* The smallest pivot over the largest entry, a cheap guard that refuses a singular matrix
 * rather than returning numbers that mean nothing. */
static int i_lu_singular(const double *lu, int n, double scale){
    int i;
    for (i = 0; i < n; i++)
        if (!(fabs(lu[i * n + i]) > 1e-14 * scale * n)) return 1;
    return 0;
}

/* x = a^-1 b for the LU of a and the k columns of b, row major n by k. */
static void i_lu_solve(const double *lu, const int *perm, int n, const double *b, double *x, int k){
    int i, j, c;
    for (c = 0; c < k; c++){
        for (i = 0; i < n; i++){
            double s = b[perm[i] * k + c];
            for (j = 0; j < i; j++) s -= lu[i * n + j] * x[j * k + c];
            x[i * k + c] = s;
        }
        for (i = n - 1; i >= 0; i--){
            double s = x[i * k + c];
            for (j = i + 1; j < n; j++) s -= lu[i * n + j] * x[j * k + c];
            x[i * k + c] = s / lu[i * n + i];
        }
    }
}

/* The inverse of the n by n a into out, with scratch of n * n doubles and n ints. */
static int i_inverse(const double *a, int n, double *out, double *lu, int *perm){
    int i, j;
    double scale = i_max_abs(a, (size_t)n * n);
    memcpy(lu, a, sizeof(double) * n * n);
    if (!i_lu(lu, n, perm) || i_lu_singular(lu, n, scale)) return 0;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++) out[i * n + j] = i == j ? 1.0 : 0.0;
    /* the identity is its own right side, solved column by column in place */
    {   int c;
        for (c = 0; c < n; c++){
            for (i = 0; i < n; i++){
                double s = perm[i] == c ? 1.0 : 0.0;
                for (j = 0; j < i; j++) s -= lu[i * n + j] * out[j * n + c];
                out[i * n + c] = s;
            }
            for (i = n - 1; i >= 0; i--){
                double s = out[i * n + c];
                for (j = i + 1; j < n; j++) s -= lu[i * n + j] * out[j * n + c];
                out[i * n + c] = s / lu[i * n + i];
            }
        }
    }
    return 1;
}

/* Cyclic Jacobi on the symmetric n by n a, in place: a ends diagonal and v holds the
 * eigenvectors as columns. Values come out ascending. */
static void i_jacobi_eigen(double *a, double *v, int n, double *values){
    int i, j, k, p, q, sweep;
    for (i = 0; i < n; i++) for (j = 0; j < n; j++) v[i * n + j] = i == j ? 1.0 : 0.0;
    for (sweep = 0; sweep < 100; sweep++){
        double off = 0.0, all = 0.0;
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++){
                double e = a[i * n + j] * a[i * n + j];
                all += e;
                if (i != j) off += e;
            }
        if (off == 0.0 || off <= 1e-32 * all) break;
        for (p = 0; p < n - 1; p++){
            for (q = p + 1; q < n; q++){
                double apq = a[p * n + q], theta, t, c, s;
                if (apq == 0.0) continue;
                theta = (a[q * n + q] - a[p * n + p]) / (2.0 * apq);
                t = (theta >= 0.0 ? 1.0 : -1.0) / (fabs(theta) + sqrt(theta * theta + 1.0));
                c = 1.0 / sqrt(t * t + 1.0);
                s = t * c;
                for (k = 0; k < n; k++){
                    double akp = a[k * n + p], akq = a[k * n + q];
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + q] = s * akp + c * akq;
                }
                for (k = 0; k < n; k++){
                    double apk = a[p * n + k], aqk = a[q * n + k];
                    a[p * n + k] = c * apk - s * aqk;
                    a[q * n + k] = s * apk + c * aqk;
                }
                for (k = 0; k < n; k++){
                    double vkp = v[k * n + p], vkq = v[k * n + q];
                    v[k * n + p] = c * vkp - s * vkq;
                    v[k * n + q] = s * vkp + c * vkq;
                }
            }
        }
    }
    for (i = 0; i < n; i++) values[i] = a[i * n + i];
    /* selection sort of values with their columns, n is small wherever this runs */
    for (i = 0; i < n - 1; i++){
        int m = i;
        for (j = i + 1; j < n; j++) if (values[j] < values[m]) m = j;
        if (m != i){
            double t = values[i]; values[i] = values[m]; values[m] = t;
            for (k = 0; k < n; k++){ t = v[k * n + i]; v[k * n + i] = v[k * n + m]; v[k * n + m] = t; }
        }
    }
}

void i_math_mat3_eigen(const i_MathMat3 *a, double values[3], i_MathMat3 *vectors){
    double w[9];
    memcpy(w, a->m, sizeof w);
    i_jacobi_eigen(w, &vectors->m[0][0], 3, values);
}

/* One sided Jacobi on the m by n u (m >= n), in place, with v n by n. u ends as the left
 * singular vectors scaled by the singular values. */
static void i_jacobi_svd(double *u, double *v, int m, int n){
    int i, p, q, sweep;
    for (p = 0; p < n; p++) for (q = 0; q < n; q++) v[p * n + q] = p == q ? 1.0 : 0.0;
    for (sweep = 0; sweep < 100; sweep++){
        int turned = 0;
        for (p = 0; p < n - 1; p++){
            for (q = p + 1; q < n; q++){
                double alpha = 0.0, beta = 0.0, gamma = 0.0, zeta, t, c, s;
                for (i = 0; i < m; i++){
                    double up = u[i * n + p], uq = u[i * n + q];
                    alpha += up * up; beta += uq * uq; gamma += up * uq;
                }
                if (gamma == 0.0 || fabs(gamma) <= 1e-15 * sqrt(alpha * beta)) continue;
                turned = 1;
                zeta = (beta - alpha) / (2.0 * gamma);
                t = (zeta >= 0.0 ? 1.0 : -1.0) / (fabs(zeta) + sqrt(1.0 + zeta * zeta));
                c = 1.0 / sqrt(1.0 + t * t);
                s = c * t;
                for (i = 0; i < m; i++){
                    double up = u[i * n + p], uq = u[i * n + q];
                    u[i * n + p] = c * up - s * uq;
                    u[i * n + q] = s * up + c * uq;
                }
                for (i = 0; i < n; i++){
                    double vp = v[i * n + p], vq = v[i * n + q];
                    v[i * n + p] = c * vp - s * vq;
                    v[i * n + q] = s * vp + c * vq;
                }
            }
        }
        if (!turned) break;
    }
}

/* The fixed size matrices. */

RantMatrix3x3 rant_matrix3x3_identity(void){
    RantMatrix3x3 m;
    int i, j;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) m.m[i][j] = i == j ? 1.0f : 0.0f;
    return m;
}

RantMatrix3x3 rant_matrix3x3_multiply(RantMatrix3x3 a, RantMatrix3x3 b){
    RantMatrix3x3 r;
    int i, j, k;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++){
            double s = 0.0;
            for (k = 0; k < 3; k++) s += (double)a.m[i][k] * b.m[k][j];
            r.m[i][j] = (float)s;
        }
    return r;
}

RantMatrix3x3 rant_matrix3x3_transpose(RantMatrix3x3 m){
    RantMatrix3x3 r;
    int i, j;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) r.m[i][j] = m.m[j][i];
    return r;
}

static void i_from_f32(const float *src, double *dst, int count){
    int i;
    for (i = 0; i < count; i++) dst[i] = src[i];
}

double rant_matrix3x3_determinant(RantMatrix3x3 m){
    double a[9], lu[9];
    int perm[3], sign;
    i_from_f32(&m.m[0][0], a, 9);
    memcpy(lu, a, sizeof lu);
    sign = i_lu(lu, 3, perm);
    return sign ? i_lu_determinant(lu, 3, sign) : 0.0;
}

RantMathResult rant_matrix3x3_inverse(RantMatrix3x3 m, RantMatrix3x3 *out){
    double a[9], lu[9], inv[9];
    int perm[3], i;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    i_from_f32(&m.m[0][0], a, 9);
    if (!i_inverse(a, 3, inv, lu, perm)) return RANT_MATH_ERR_DEGENERATE;
    for (i = 0; i < 9; i++) (&out->m[0][0])[i] = (float)inv[i];
    return RANT_MATH_OK;
}

RantDouble3 rant_matrix3x3_apply(RantMatrix3x3 m, RantDouble3 v){
    return rant_double3((double)m.m[0][0] * v.x + (double)m.m[0][1] * v.y + (double)m.m[0][2] * v.z,
                        (double)m.m[1][0] * v.x + (double)m.m[1][1] * v.y + (double)m.m[1][2] * v.z,
                        (double)m.m[2][0] * v.x + (double)m.m[2][1] * v.y + (double)m.m[2][2] * v.z);
}

RantMatrix4x4 rant_matrix4x4_identity(void){
    RantMatrix4x4 m;
    int i, j;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) m.m[i][j] = i == j ? 1.0f : 0.0f;
    return m;
}

RantMatrix4x4 rant_matrix4x4_multiply(RantMatrix4x4 a, RantMatrix4x4 b){
    RantMatrix4x4 r;
    int i, j, k;
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++){
            double s = 0.0;
            for (k = 0; k < 4; k++) s += (double)a.m[i][k] * b.m[k][j];
            r.m[i][j] = (float)s;
        }
    return r;
}

RantMatrix4x4 rant_matrix4x4_transpose(RantMatrix4x4 m){
    RantMatrix4x4 r;
    int i, j;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) r.m[i][j] = m.m[j][i];
    return r;
}

double rant_matrix4x4_determinant(RantMatrix4x4 m){
    double lu[16];
    int perm[4], sign;
    i_from_f32(&m.m[0][0], lu, 16);
    sign = i_lu(lu, 4, perm);
    return sign ? i_lu_determinant(lu, 4, sign) : 0.0;
}

RantMathResult rant_matrix4x4_inverse(RantMatrix4x4 m, RantMatrix4x4 *out){
    double a[16], lu[16], inv[16];
    int perm[4], i;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    i_from_f32(&m.m[0][0], a, 16);
    if (!i_inverse(a, 4, inv, lu, perm)) return RANT_MATH_ERR_DEGENERATE;
    for (i = 0; i < 16; i++) (&out->m[0][0])[i] = (float)inv[i];
    return RANT_MATH_OK;
}

RantDouble3 rant_matrix4x4_apply(RantMatrix4x4 m, RantDouble3 p){
    double r[4];
    int i;
    for (i = 0; i < 4; i++)
        r[i] = (double)m.m[i][0] * p.x + (double)m.m[i][1] * p.y + (double)m.m[i][2] * p.z + (double)m.m[i][3];
    if (r[3] != 1.0 && r[3] != 0.0) return rant_double3(r[0] / r[3], r[1] / r[3], r[2] / r[3]);
    return rant_double3(r[0], r[1], r[2]);
}

RantDouble3 rant_matrix4x4_apply_direction(RantMatrix4x4 m, RantDouble3 d){
    return rant_double3((double)m.m[0][0] * d.x + (double)m.m[0][1] * d.y + (double)m.m[0][2] * d.z,
                        (double)m.m[1][0] * d.x + (double)m.m[1][1] * d.y + (double)m.m[1][2] * d.z,
                        (double)m.m[2][0] * d.x + (double)m.m[2][1] * d.y + (double)m.m[2][2] * d.z);
}

/* The dense matrices. */

static int i_valid(RantMatrixView a){
    return a.data && a.rows > 0 && a.cols > 0 && a.rows <= 65535u && a.cols <= 65535u;
}

static size_t i_count(RantMatrixView a){ return (size_t)a.rows * a.cols; }

static int i_overlap(RantMatrixView a, RantMatrixView b){
    const double *a0 = a.data, *a1 = a.data + i_count(a);
    const double *b0 = b.data, *b1 = b.data + i_count(b);
    return a0 < b1 && b0 < a1;
}

RantMathResult rant_matrix_multiply(RantMatrixView a, RantMatrixView b, RantMatrixView out){
    uint32_t i, j, k;
    if (!i_valid(a) || !i_valid(b) || !i_valid(out)) return RANT_MATH_ERR_ARGUMENT;
    if (a.cols != b.rows || out.rows != a.rows || out.cols != b.cols) return RANT_MATH_ERR_ARGUMENT;
    if (i_overlap(a, out) || i_overlap(b, out)) return RANT_MATH_ERR_ARGUMENT;
    for (i = 0; i < a.rows; i++)
        for (j = 0; j < b.cols; j++){
            double s = 0.0;
            for (k = 0; k < a.cols; k++) s += a.data[(size_t)i * a.cols + k] * b.data[(size_t)k * b.cols + j];
            out.data[(size_t)i * out.cols + j] = s;
        }
    return RANT_MATH_OK;
}

RantMathResult rant_matrix_transpose(RantMatrixView a, RantMatrixView out){
    uint32_t i, j;
    if (!i_valid(a) || !i_valid(out) || out.rows != a.cols || out.cols != a.rows || i_overlap(a, out))
        return RANT_MATH_ERR_ARGUMENT;
    for (i = 0; i < a.rows; i++)
        for (j = 0; j < a.cols; j++) out.data[(size_t)j * out.cols + i] = a.data[(size_t)i * a.cols + j];
    return RANT_MATH_OK;
}

RantMathResult rant_matrix_determinant(RantMatrixView a, double *out){
    double *lu;
    int *perm, sign, n = (int)a.rows;
    if (!i_valid(a) || a.rows != a.cols || !out) return RANT_MATH_ERR_ARGUMENT;
    lu = (double *)malloc(sizeof(double) * i_count(a));
    perm = (int *)malloc(sizeof(int) * a.rows);
    if (!lu || !perm){ free(lu); free(perm); return RANT_MATH_ERR_MEMORY; }
    memcpy(lu, a.data, sizeof(double) * i_count(a));
    sign = i_lu(lu, n, perm);
    *out = sign ? i_lu_determinant(lu, n, sign) : 0.0;
    free(lu); free(perm);
    return RANT_MATH_OK;
}

RantMathResult rant_matrix_inverse(RantMatrixView a, RantMatrixView out){
    double *lu;
    int *perm, ok;
    if (!i_valid(a) || !i_valid(out) || a.rows != a.cols || out.rows != a.rows || out.cols != a.cols
        || i_overlap(a, out)) return RANT_MATH_ERR_ARGUMENT;
    lu = (double *)malloc(sizeof(double) * i_count(a));
    perm = (int *)malloc(sizeof(int) * a.rows);
    if (!lu || !perm){ free(lu); free(perm); return RANT_MATH_ERR_MEMORY; }
    ok = i_inverse(a.data, (int)a.rows, out.data, lu, perm);
    free(lu); free(perm);
    return ok ? RANT_MATH_OK : RANT_MATH_ERR_DEGENERATE;
}

/* Householder QR of the m by n w (m >= n) applied to the m by k right side c, then back
 * substitution of the top n rows. Rank deficient columns fail. */
static int i_qr_solve(double *w, int m, int n, double *c, int k, double *x){
    int i, j, col, p;
    double scale = i_max_abs(w, (size_t)m * n);
    for (j = 0; j < n; j++){
        double norm = 0.0, alpha, vnorm2;
        for (i = j; i < m; i++) norm += w[i * n + j] * w[i * n + j];
        norm = sqrt(norm);
        if (!(norm > 1e-14 * scale * m)) return 0;
        alpha = w[j * n + j] > 0.0 ? -norm : norm;
        /* v = column j below the diagonal with v_j = w_jj - alpha, kept in place */
        w[j * n + j] -= alpha;
        vnorm2 = 0.0;
        for (i = j; i < m; i++) vnorm2 += w[i * n + j] * w[i * n + j];
        for (col = j + 1; col < n; col++){
            double d = 0.0;
            for (i = j; i < m; i++) d += w[i * n + j] * w[i * n + col];
            d = 2.0 * d / vnorm2;
            for (i = j; i < m; i++) w[i * n + col] -= d * w[i * n + j];
        }
        for (p = 0; p < k; p++){
            double d = 0.0;
            for (i = j; i < m; i++) d += w[i * n + j] * c[i * k + p];
            d = 2.0 * d / vnorm2;
            for (i = j; i < m; i++) c[i * k + p] -= d * w[i * n + j];
        }
        /* the reflected column is alpha on the diagonal and zero below */
        w[j * n + j] = alpha;
    }
    for (p = 0; p < k; p++)
        for (i = n - 1; i >= 0; i--){
            double s = c[i * k + p];
            for (j = i + 1; j < n; j++) s -= w[i * n + j] * x[j * k + p];
            x[i * k + p] = s / w[i * n + i];
        }
    return 1;
}

RantMathResult rant_matrix_solve(RantMatrixView a, RantMatrixView b, RantMatrixView x){
    int ok;
    if (!i_valid(a) || !i_valid(b) || !i_valid(x)) return RANT_MATH_ERR_ARGUMENT;
    if (b.rows != a.rows || x.rows != a.cols || x.cols != b.cols || a.rows < a.cols)
        return RANT_MATH_ERR_ARGUMENT;
    if (i_overlap(a, x) || i_overlap(b, x)) return RANT_MATH_ERR_ARGUMENT;
    if (a.rows == a.cols){
        double *lu = (double *)malloc(sizeof(double) * i_count(a));
        int *perm = (int *)malloc(sizeof(int) * a.rows);
        if (!lu || !perm){ free(lu); free(perm); return RANT_MATH_ERR_MEMORY; }
        memcpy(lu, a.data, sizeof(double) * i_count(a));
        ok = i_lu(lu, (int)a.rows, perm) && !i_lu_singular(lu, (int)a.rows, i_max_abs(a.data, i_count(a)));
        if (ok) i_lu_solve(lu, perm, (int)a.rows, b.data, x.data, (int)b.cols);
        free(lu); free(perm);
    } else {
        double *w = (double *)malloc(sizeof(double) * i_count(a));
        double *c = (double *)malloc(sizeof(double) * i_count(b));
        if (!w || !c){ free(w); free(c); return RANT_MATH_ERR_MEMORY; }
        memcpy(w, a.data, sizeof(double) * i_count(a));
        memcpy(c, b.data, sizeof(double) * i_count(b));
        ok = i_qr_solve(w, (int)a.rows, (int)a.cols, c, (int)b.cols, x.data);
        free(w); free(c);
    }
    return ok ? RANT_MATH_OK : RANT_MATH_ERR_DEGENERATE;
}

/* Fill column j of the m by k u with a unit vector at right angles to every other column,
 * for a zero singular value whose direction Jacobi leaves empty. */
static void i_complete_column(double *u, int m, int k, int j){
    int e, i, c;
    for (e = 0; e < m; e++){
        double norm = 0.0;
        for (i = 0; i < m; i++) u[i * k + j] = i == e ? 1.0 : 0.0;
        for (c = 0; c < k; c++){
            double d = 0.0;
            if (c == j) continue;
            for (i = 0; i < m; i++) d += u[i * k + c] * u[i * k + j];
            for (i = 0; i < m; i++) u[i * k + j] -= d * u[i * k + c];
        }
        for (i = 0; i < m; i++) norm += u[i * k + j] * u[i * k + j];
        norm = sqrt(norm);
        if (norm > 1e-6){
            for (i = 0; i < m; i++) u[i * k + j] /= norm;
            return;
        }
    }
}

RantMathResult rant_matrix_svd(RantMatrixView a, RantMatrixView u, double *s, RantMatrixView v){
    int m = (int)a.rows, n = (int)a.cols, tall = m >= n, rows, cols, i, j, k;
    double *w, *vv;
    if (!i_valid(a) || !i_valid(u) || !i_valid(v) || !s) return RANT_MATH_ERR_ARGUMENT;
    k = m < n ? m : n;
    if ((int)u.rows != m || (int)u.cols != k || (int)v.rows != n || (int)v.cols != k)
        return RANT_MATH_ERR_ARGUMENT;
    if (i_overlap(a, u) || i_overlap(a, v) || i_overlap(u, v)) return RANT_MATH_ERR_ARGUMENT;
    /* a wide matrix is solved as its transpose with u and v trading places */
    rows = tall ? m : n;
    cols = tall ? n : m;
    w = (double *)malloc(sizeof(double) * (size_t)rows * cols);
    vv = (double *)malloc(sizeof(double) * (size_t)cols * cols);
    if (!w || !vv){ free(w); free(vv); return RANT_MATH_ERR_MEMORY; }
    for (i = 0; i < rows; i++)
        for (j = 0; j < cols; j++) w[i * cols + j] = tall ? a.data[i * n + j] : a.data[j * n + i];
    i_jacobi_svd(w, vv, rows, cols);
    {   double *sv = s;
        int *order = (int *)malloc(sizeof(int) * cols);
        double *left = tall ? u.data : v.data, *right = tall ? v.data : u.data;
        if (!order){ free(w); free(vv); return RANT_MATH_ERR_MEMORY; }
        for (j = 0; j < cols; j++){
            double norm = 0.0;
            for (i = 0; i < rows; i++) norm += w[i * cols + j] * w[i * cols + j];
            sv[j] = sqrt(norm);
            order[j] = j;
        }
        for (i = 0; i < cols - 1; i++)
            for (j = i + 1; j < cols; j++)
                if (sv[order[j]] > sv[order[i]]){ int t = order[i]; order[i] = order[j]; order[j] = t; }
        {   double *sorted = (double *)malloc(sizeof(double) * cols);
            if (!sorted){ free(order); free(w); free(vv); return RANT_MATH_ERR_MEMORY; }
            for (j = 0; j < cols; j++) sorted[j] = sv[order[j]];
            for (j = 0; j < cols; j++){
                int src = order[j];
                for (i = 0; i < rows; i++)
                    left[i * cols + j] = sorted[j] > 0.0 ? w[i * cols + src] / sorted[j] : 0.0;
                for (i = 0; i < cols; i++) right[i * cols + j] = vv[i * cols + src];
            }
            for (j = 0; j < cols; j++){
                sv[j] = sorted[j];
                if (!(sorted[j] > 1e-300)) i_complete_column(left, rows, cols, j);
            }
            free(sorted);
        }
        free(order);
    }
    free(w); free(vv);
    return RANT_MATH_OK;
}

RantMathResult rant_matrix_eigen(RantMatrixView a, double *values, RantMatrixView vectors){
    int n = (int)a.rows, i, j;
    double *w, scale;
    if (!i_valid(a) || !i_valid(vectors) || !values || a.rows != a.cols
        || vectors.rows != a.rows || vectors.cols != a.cols || i_overlap(a, vectors))
        return RANT_MATH_ERR_ARGUMENT;
    scale = i_max_abs(a.data, i_count(a));
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++)
            if (fabs(a.data[i * n + j] - a.data[j * n + i]) > 1e-9 * scale) return RANT_MATH_ERR_ARGUMENT;
    w = (double *)malloc(sizeof(double) * i_count(a));
    if (!w) return RANT_MATH_ERR_MEMORY;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++) w[i * n + j] = 0.5 * (a.data[i * n + j] + a.data[j * n + i]);
    i_jacobi_eigen(w, vectors.data, n, values);
    free(w);
    return RANT_MATH_OK;
}

RantMathResult rant_matrix_cholesky(RantMatrixView a, RantMatrixView l){
    int n = (int)a.rows, i, j, k;
    double scale;
    if (!i_valid(a) || !i_valid(l) || a.rows != a.cols || l.rows != a.rows || l.cols != a.cols
        || i_overlap(a, l)) return RANT_MATH_ERR_ARGUMENT;
    scale = i_max_abs(a.data, i_count(a));
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++)
            if (fabs(a.data[i * n + j] - a.data[j * n + i]) > 1e-9 * scale) return RANT_MATH_ERR_ARGUMENT;
    {   double *w = (double *)calloc(i_count(a), sizeof(double));
        if (!w) return RANT_MATH_ERR_MEMORY;
        for (j = 0; j < n; j++){
            double d = a.data[j * n + j];
            for (k = 0; k < j; k++) d -= w[j * n + k] * w[j * n + k];
            if (!(d > 1e-14 * scale)){ free(w); return RANT_MATH_ERR_DEGENERATE; }
            w[j * n + j] = sqrt(d);
            for (i = j + 1; i < n; i++){
                double s = a.data[i * n + j];
                for (k = 0; k < j; k++) s -= w[i * n + k] * w[j * n + k];
                w[i * n + j] = s / w[j * n + j];
            }
        }
        memcpy(l.data, w, sizeof(double) * i_count(a));
        free(w);
    }
    return RANT_MATH_OK;
}
