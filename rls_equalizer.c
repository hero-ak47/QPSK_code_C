#include "rls_equalizer.h"
#include <math.h>

// So sanh voi 4 diem QPSK chuan de tim symbol gan nhat (hard decision)
// constellation: {1+1j, -1+1j, -1-1j, 1-1j} tuong duong sign(real)+j*sign(imag)
static cplx_t hard_decision_qpsk(cplx_t x) {
    float re = (x.re >= 0.0f) ? 1.0f : -1.0f;
    float im = (x.im >= 0.0f) ? 1.0f : -1.0f;
    return cplx_make(re, im);
}

// Port cua rls_equalize_zc (scalar RLS, 1 tap W)
// rx_payload_cfo: dau vao (da bu CFO)
// pilot_indices: chi so pilot (0-based), n_pilots: so pilot
// zc_pilots: chuoi pilot chuan
// data_indices: chi so data (0-based), n_data: so luong data symbol
// out_eq: output full equalized (cung do dai voi rx_payload_cfo) - optional, co the NULL
// out_data_eq: output chi cac symbol du lieu (dai n_data) - CAN cho buoc demod QPSK sau
// lambda_warmup, lambda_main, n_warmup: tham so RLS
void rls_equalize_zc(const cplx_t *rx_payload_cfo, int payload_len,
                       const int *pilot_indices, int n_pilots,
                       const cplx_t *zc_pilots,
                       const int *data_indices, int n_data,
                       cplx_t *out_eq, cplx_t *out_data_eq,
                       float lambda_warmup, float lambda_main,
                       float P_init, cplx_t W_init, int n_warmup) {

    cplx_t W = W_init;
    float P = P_init;

    // ---- Warm-up: dung pilot dau tien lap lai n_warmup lan ----
    cplx_t first_pilot_rx = rx_payload_cfo[pilot_indices[0]];
    cplx_t first_pilot_tx = zc_pilots[0];

    for (int it = 0; it < n_warmup; it++) {
        cplx_t y_wu = first_pilot_rx;
        cplx_t d_wu = first_pilot_tx;

        float y_mag_sq = cplx_abs2(y_wu);
        // k = (P * conj(y)) / (lambda + P*|y|^2)
        cplx_t num_k = cplx_scale(cplx_conj(y_wu), P);
        float denom_k = lambda_warmup + P*y_mag_sq;
        cplx_t k = cplx_scale(num_k, 1.0f/denom_k);

        // e = d - W*y
        cplx_t Wy = cplx_mul(W, y_wu);
        cplx_t e = cplx_sub(d_wu, Wy);

        // W = W + k*e
        W = cplx_add(W, cplx_mul(k, e));

        // P = (P - k*y*P) / lambda   (chu y: k*y la so phuc, P la so thuc -
        //     trong MATLAB P van la scalar thuc vi day la RLS 1 tap don gian hoa)
        cplx_t ky = cplx_mul(k, y_wu);
        // Lay phan thuc vi P dinh nghia la scalar thuc trong ban goc MATLAB
        // (dam bao dung nhu logic goc: P = (P - k*y*P)/lambda, neu k*y co phan ao
        //  MATLAB van cho phep P thanh phuc - nhung o day gia dinh dung nhu ban goc,
        //  neu can chinh xac tuyet doi, doi P sang cplx_t)
        float ky_re = ky.re;  // dung phan thuc lam xap xi (xem ghi chu duoi)
        P = (P - ky_re*P) / lambda_warmup;
    }

    // ---- Vong lap chinh: chay qua toan bo payload ----
    int pilot_cnt = 0;
    for (int i = 0; i < payload_len; i++) {
        cplx_t y_k = rx_payload_cfo[i];
        cplx_t d_k;

        // Kiem tra i co phai la pilot khong
        int is_pilot = 0;
        for (int p = 0; p < n_pilots; p++) {
            if (pilot_indices[p] == i) { is_pilot = 1; break; }
        }

        if (is_pilot) {
            d_k = zc_pilots[pilot_cnt];
            pilot_cnt++;
        } else {
            cplx_t eq_temp = cplx_mul(W, y_k);
            d_k = hard_decision_qpsk(eq_temp);
        }

        cplx_t eq_out = cplx_mul(W, y_k);
        if (out_eq != NULL) out_eq[i] = eq_out;

        float y_mag_sq = cplx_abs2(y_k);
        cplx_t num_k = cplx_scale(cplx_conj(y_k), P);
        float denom_k = lambda_main + P*y_mag_sq;
        cplx_t k = cplx_scale(num_k, 1.0f/denom_k);

        cplx_t e = cplx_sub(d_k, eq_out);
        W = cplx_add(W, cplx_mul(k, e));

        cplx_t ky = cplx_mul(k, y_k);
        float ky_re = ky.re;
        P = (P - ky_re*P) / lambda_main;
    }

    // ---- Trich xuat cac symbol data (khong phai pilot) tu out_eq ----
    if (out_eq != NULL && out_data_eq != NULL) {
        for (int d = 0; d < n_data; d++) {
            out_data_eq[d] = out_eq[data_indices[d]];
        }
    }
}
