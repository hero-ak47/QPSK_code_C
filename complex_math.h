#ifndef COMPLEX_MATH_H
#define COMPLEX_MATH_H

#include <math.h>

// So phuc tu dinh nghia (khong dung <complex.h> de kiem soat ro rang tren bare-metal,
// tranh phu thuoc libm phuc tap khong can thiet tren embedded)
typedef struct {
    float re;
    float im;
} cplx_t;

static inline cplx_t cplx_make(float re, float im) {
    cplx_t c; c.re = re; c.im = im; return c;
}

static inline cplx_t cplx_add(cplx_t a, cplx_t b) {
    return cplx_make(a.re + b.re, a.im + b.im);
}

static inline cplx_t cplx_sub(cplx_t a, cplx_t b) {
    return cplx_make(a.re - b.re, a.im - b.im);
}

static inline cplx_t cplx_mul(cplx_t a, cplx_t b) {
    return cplx_make(a.re*b.re - a.im*b.im, a.re*b.im + a.im*b.re);
}

static inline cplx_t cplx_scale(cplx_t a, float s) {
    return cplx_make(a.re * s, a.im * s);
}

// Lien hop phuc: conj(a) = a.re - j*a.im
static inline cplx_t cplx_conj(cplx_t a) {
    return cplx_make(a.re, -a.im);
}

// Chia so phuc: a / b
static inline cplx_t cplx_div(cplx_t a, cplx_t b) {
    float denom = b.re*b.re + b.im*b.im;
    if (denom < 1e-20f) denom = 1e-20f; // tranh chia 0
    cplx_t num = cplx_mul(a, cplx_conj(b));
    return cplx_scale(num, 1.0f/denom);
}

static inline float cplx_abs(cplx_t a) {
    return sqrtf(a.re*a.re + a.im*a.im);
}

static inline float cplx_abs2(cplx_t a) {  // |a|^2, nhanh hon vi khong can sqrt
    return a.re*a.re + a.im*a.im;
}

static inline float cplx_angle(cplx_t a) {
    return atan2f(a.im, a.re);
}

// exp(j*theta) = cos(theta) + j*sin(theta)
static inline cplx_t cplx_expj(float theta) {
    return cplx_make(cosf(theta), sinf(theta));
}

#endif // COMPLEX_MATH_H
