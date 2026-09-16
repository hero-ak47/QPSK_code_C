#include "cfo_estimate.h"
#include "config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Unwrap pha (thay ham unwrap cua MATLAB) - lam viec tren mang phase (radian)
static void unwrap_phase(float *phase, int len) {
    for (int i = 1; i < len; i++) {
        float diff = phase[i] - phase[i-1];
        while (diff > M_PI) {
            phase[i] -= 2.0f*M_PI;
            diff = phase[i] - phase[i-1];
        }
        while (diff < -M_PI) {
            phase[i] += 2.0f*M_PI;
            diff = phase[i] - phase[i-1];
        }
    }
}

// Port cua estimate_and_correct_cfo trong MATLAB
// rx_payload: mang complex dau vao (se bi ghi de bang du lieu da bu CFO)
// pilot_indices: chi so pilot (0-based, khac voi MATLAB 1-based - da tru san khi goi ham)
// zc_pilots: chuoi pilot ZC chuan
// n_pilots: so luong pilot
// payload_len: tong do dai rx_payload
// L: samples per symbol, fs: sample rate
// Tra ve: cfo_est (Hz)
float estimate_and_correct_cfo(cplx_t *rx_payload, int payload_len,
                                 const int *pilot_indices, int n_pilots,
                                 const cplx_t *zc_pilots,
                                 int L, float fs) {

    float pilot_phase_raw[N_PILOTS];       // gia su N_PILOTS du lon (dinh nghia trong config.h)
    float pilot_phase_unwrapped[N_PILOTS];
    float idx_col[N_PILOTS];

    // pilot_phase_raw[i] = angle(rx_pilots_raw[i] / zc_pilots[i])
    for (int i = 0; i < n_pilots; i++) {
        cplx_t rx_p = rx_payload[pilot_indices[i]];
        cplx_t ratio = cplx_div(rx_p, zc_pilots[i]);
        pilot_phase_raw[i] = cplx_angle(ratio);
        idx_col[i] = (float)pilot_indices[i];  // da 0-based (MATLAB dung pilot_indices-1)
    }

    // Sao chep de unwrap (khong pha huy ban goc)
    for (int i = 0; i < n_pilots; i++) pilot_phase_unwrapped[i] = pilot_phase_raw[i];
    unwrap_phase(pilot_phase_unwrapped, n_pilots);

    // Linear least-squares fit: phase = c0 + c1*idx  (tuong duong A\pilot_phase_unwrapped trong MATLAB)
    // Dung cong thuc closed-form cho hoi quy tuyen tinh 1 bien:
    float sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    for (int i = 0; i < n_pilots; i++) {
        sum_x  += idx_col[i];
        sum_y  += pilot_phase_unwrapped[i];
        sum_xx += idx_col[i]*idx_col[i];
        sum_xy += idx_col[i]*pilot_phase_unwrapped[i];
    }
    float n = (float)n_pilots;
    float denom = n*sum_xx - sum_x*sum_x;
    float slope = (n*sum_xy - sum_x*sum_y) / denom;   // c1 (rad/symbol)
    // float intercept = (sum_y - slope*sum_x) / n;   // c0, khong can dung tiep

    float Tsym = (float)L / fs;
    float cfo_est = slope / (2.0f*M_PI*Tsym);

    // Bu CFO: rx_payload_cfo[n] = rx_payload[n] * exp(-j*2*pi*cfo_est*Tsym*n)
    for (int n_idx = 0; n_idx < payload_len; n_idx++) {
        float phase_corr = -2.0f*M_PI*cfo_est*Tsym*(float)n_idx;
        cplx_t corr = cplx_expj(phase_corr);
        rx_payload[n_idx] = cplx_mul(rx_payload[n_idx], corr);
    }

    return cfo_est;
}
