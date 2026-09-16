#include "rrc_filter.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Thay the ham rcosdesign(beta, span, L, 'sqrt') cua MATLAB.
// Tra ve h_rrc co do dai N_TAPS = span*L + 1
void rrc_design(float *h, int span, int L, float beta) {
    int N = span * L + 1;
    int mid = N / 2;

    for (int i = 0; i < N; i++) {
        // t tinh theo don vi symbol (giong cach rcosdesign chuan hoa)
        float t = (float)(i - mid) / (float)L;

        if (fabsf(t) < 1e-8f) {
            // t = 0
            h[i] = (1.0f - beta) + 4.0f*beta/M_PI;
        }
        else if (fabsf(fabsf(4.0f*beta*t) - 1.0f) < 1e-6f) {
            // t = +-1/(4*beta) : cong thuc gioi han (L'Hopital)
            float term1 = (1.0f + 2.0f/M_PI) * sinf(M_PI/(4.0f*beta));
            float term2 = (1.0f - 2.0f/M_PI) * cosf(M_PI/(4.0f*beta));
            h[i] = (beta/sqrtf(2.0f)) * (term1 + term2);
        }
        else {
            float numerator = sinf(M_PI*t*(1.0f-beta)) +
                               4.0f*beta*t*cosf(M_PI*t*(1.0f+beta));
            float denominator = M_PI*t*(1.0f - (4.0f*beta*t)*(4.0f*beta*t));
            h[i] = numerator / denominator;
        }
    }

    // Chuan hoa nang luong (giong rcosdesign mac dinh chuan hoa gain=1 tai DC sau normalize)
    float energy = 0.0f;
    for (int i = 0; i < N; i++) energy += h[i]*h[i];
    float norm = sqrtf(energy);
    if (norm > 1e-12f) {
        for (int i = 0; i < N; i++) h[i] /= norm;
    }
}

// Thay the upfirdn(x, h, L): upsample x len L lan (chen L-1 so 0 giua cac mau),
// sau do loc qua FIR h. Dung cho viec tai tao preamble_bb_ideal lam mau so sanh.
// out phai duoc cap phat truoc voi kich thuoc >= (in_len*L + h_len - 1)
int upfirdn_upsample(const cplx_t *in, int in_len, const float *h, int h_len,
                       int L, cplx_t *out) {
    int up_len = in_len * L;
    int out_len = up_len + h_len - 1;

    // Buoc 1: upsample (chen zero) - lam tam trong out truoc, se convolve sau
    // De tiet kiem bo nho embedded, ta convolve truc tiep khong tao mang up rieng:
    for (int n = 0; n < out_len; n++) {
        cplx_t acc = cplx_make(0.0f, 0.0f);
        for (int k = 0; k < h_len; k++) {
            int up_idx = n - k;
            if (up_idx < 0 || up_idx >= up_len) continue;
            if (up_idx % L != 0) continue;  // chi cac vi tri la boi cua L moi co gia tri (con lai la 0 da chen)
            int in_idx = up_idx / L;
            if (in_idx < 0 || in_idx >= in_len) continue;
            cplx_t sample = cplx_scale(in[in_idx], h[k]);
            acc = cplx_add(acc, sample);
        }
        out[n] = acc;
    }
    return out_len;
}

// FIR filter thuong (thay ham filter() cua MATLAB, chi dung FIR - IIR nhu butter can rieng)
// out co cung do dai voi in (dung 'same' padding giong conv(...,'same'))
void fir_filter_same(const cplx_t *in, int in_len, const float *h, int h_len,
                       cplx_t *out) {
    int half = h_len / 2;
    for (int n = 0; n < in_len; n++) {
        cplx_t acc = cplx_make(0.0f, 0.0f);
        for (int k = 0; k < h_len; k++) {
            int idx = n - k + half;
            if (idx < 0 || idx >= in_len) continue;
            cplx_t sample = cplx_scale(in[idx], h[k]);
            acc = cplx_add(acc, sample);
        }
        out[n] = acc;
    }
}
