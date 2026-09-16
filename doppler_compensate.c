#include "doppler_compensate.h"
#include "frame_sync.h"
#include <stdlib.h>
#include <math.h>

// ============================================================================
// Uoc luong he so Doppler bang 2 preamble (dinh 1 = dau frame, dinh 2 = cuoi frame)
// Port truc tiep tu doan MATLAB:
//   T0 = N_between_symbols * L / Fs;
//   sample_diff = lag2 - lag1;
//   T_rx = sample_diff / Fs;
//   a_hat = T0/T_rx - 1;
// ============================================================================
float estimate_doppler_2pilot(const cplx_t *y, int y_len,
                                const cplx_t *pss_raw, int nzc, int L,
                                const cplx_t *preamble_up, int preamble_up_len,
                                int search_limit_lag, int N_between_symbols, float fs,
                                int *lag1_out, int *lag2_out,
                                float *peak1_out, float *peak2_out) {

    // ---- Tim dinh 1 (preamble DAU) trong nua dau buffer ----
    int lag1;
    float peak1, noise1;
    frame_sync_two_stage(y, y_len, pss_raw, nzc, L, preamble_up, preamble_up_len,
                           0, search_limit_lag, &lag1, &peak1, &noise1);

    // ---- Tim dinh 2 (preamble CUOI) trong nua sau buffer ----
    int lag2;
    float peak2, noise2;
    frame_sync_two_stage(y, y_len, pss_raw, nzc, L, preamble_up, preamble_up_len,
                           search_limit_lag + 1, y_len - 1, &lag2, &peak2, &noise2);

    *lag1_out = lag1;
    *lag2_out = lag2;
    *peak1_out = peak1;
    *peak2_out = peak2;

    // ---- Tinh he so Doppler ----
    float T0 = (float)N_between_symbols * (float)L / fs;
    int sample_diff = lag2 - lag1;
    float T_rx = (float)sample_diff / fs;

    if (fabsf(T_rx) < 1e-9f) {
        return 0.0f;  // tranh chia 0 neu dinh 2 tim sai (trung voi dinh 1)
    }

    float a_hat = T0 / T_rx - 1.0f;
    return a_hat;
}

// ============================================================================
// Bu Doppler bang resample tuyen tinh
// Port tu doan MATLAB:
//   t_orig = (0:N-1)'/Fs;
//   t_new  = t_orig / (1 + a_try);
//   y_out  = interp1(t_orig, y, t_new, 'spline', 'extrap');
//
// LUU Y: MATLAB dung 'spline' (noi suy bac 3), o day dung LINEAR de phu hop
// tai nguyen embedded. Neu can do chinh xac cao hon, co the nang cap len
// Catmull-Rom hoac cubic spline sau, nhung linear thuong du dung neu a_hat nho
// (~vai phan nghin nhu Doppler dien hinh trong kenh am thanh/vo tuyen tam gan).
// ============================================================================
int doppler_resample_linear(const cplx_t *y_in, int y_in_len,
                              float a_hat, float fs,
                              cplx_t *y_out) {

    float one_plus_a = 1.0f + a_hat;
    if (fabsf(one_plus_a) < 1e-9f) one_plus_a = 1e-9f;  // tranh chia 0 neu a_hat = -1 (khong thuc te)

    for (int i = 0; i < y_in_len; i++) {
        float t_orig = (float)i / fs;
        float t_new  = t_orig / one_plus_a;

        // Chuyen t_new tro lai thanh chi so mau (float) trong mang goc
        float idx_f = t_new * fs;

        int idx_floor = (int)floorf(idx_f);
        float frac = idx_f - (float)idx_floor;

        cplx_t sample;

        if (idx_floor < 0) {
            // Extrapolate: lay mau dau tien (tuong duong 'extrap' don gian hoa)
            sample = y_in[0];
        } else if (idx_floor >= y_in_len - 1) {
            // Extrapolate: lay mau cuoi cung
            sample = y_in[y_in_len - 1];
        } else {
            // Noi suy tuyen tinh giua idx_floor va idx_floor+1
            cplx_t a = y_in[idx_floor];
            cplx_t b = y_in[idx_floor + 1];
            sample.re = a.re + frac * (b.re - a.re);
            sample.im = a.im + frac * (b.im - a.im);
        }

        y_out[i] = sample;
    }

    return y_in_len;
}
