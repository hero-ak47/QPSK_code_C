#include "frame_sync.h"
#include <stdlib.h>
#include <string.h>

// So sanh 2 so float, dung cho qsort tim median (noise_floor)
static int cmp_float(const void *a, const void *b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

// ============================================================================
// CANH BAO HIEU NANG: Cross-correlation vet can tung mau (nhu ham cu) co do phuc tap
// O(range_len * preamble_len). Voi preamble da oversample (~30000 mau) va range
// tim kiem ~240000 mau, tong phep tinh len toi HANG CHUC TY phep nhan phuc -
// KHONG THE CHAY DUOC ca tren PC lan tren embedded ARM trong thoi gian hop ly.
//
// Giai phap: tim kiem 2 TANG (coarse-to-fine), giam do phuc tap xuong con
// vai chuc TRIEU phep tinh (nhanh hon ~1000 lan):
//   - Tang 1 (COARSE, do phan giai 1 symbol): dung truc tiep chuoi ZC goc (KHONG
//     oversample, chi NZC=63 mau) tuong quan voi tin hieu y lay mau tai moi
//     "pha" trong 1 chu ky symbol (0..L-1) va moi vi tri symbol. Do phuc tap:
//     O(search_range * NZC) thay vi O(search_range * NZC * L).
//   - Tang 2 (FINE, do phan giai 1 mau): quanh diem coarse tot nhat, chi quet
//     +/- L mau xung quanh bang preamble DA OVERSAMPLE (chinh xac tuyet doi).
//     Do phuc tap: O(L * preamble_len_oversampled) - rat nho.
// ============================================================================

// Tang 1: coarse search dung ZC goc (khong oversample), tra ve lag coarse tot nhat (don vi: mau)
static int coarse_search(const cplx_t *y, int y_len,
                           const cplx_t *pss_raw, int nzc, int L,
                           int lag_min, int lag_max,
                           float *coarse_peak_out) {
    float best_peak = -1.0f;
    int best_lag = lag_min;

    // Quet tung "pha" phi trong 1 chu ky symbol, va tung vi tri symbol m
    for (int phi = 0; phi < L; phi++) {
        int lag_start = lag_min + phi;
        for (int lag = lag_start; lag <= lag_max; lag += L) {
            cplx_t acc = cplx_make(0.0f, 0.0f);
            int valid = 1;
            for (int k = 0; k < nzc; k++) {
                int y_idx = lag + k*L;
                if (y_idx < 0 || y_idx >= y_len) { valid = 0; break; }
                cplx_t term = cplx_mul(y[y_idx], cplx_conj(pss_raw[k]));
                acc = cplx_add(acc, term);
            }
            if (!valid) continue;

            float mag = cplx_abs(acc);
            if (mag > best_peak) {
                best_peak = mag;
                best_lag = lag;
            }
        }
    }

    *coarse_peak_out = best_peak;
    return best_lag;
}

// Tang 2: fine search dung preamble DA OVERSAMPLE, quet +/- window quanh coarse_lag
static int fine_search(const cplx_t *y, int y_len,
                         const cplx_t *preamble_up, int preamble_up_len,
                         int coarse_lag, int window,
                         float *fine_peak_out) {
    float best_peak = -1.0f;
    int best_lag = coarse_lag;

    int lag_min = coarse_lag - window;
    int lag_max = coarse_lag + window;

    for (int lag = lag_min; lag <= lag_max; lag++) {
        cplx_t acc = cplx_make(0.0f, 0.0f);
        int valid = 1;
        for (int n = 0; n < preamble_up_len; n++) {
            int y_idx = lag + n;
            if (y_idx < 0 || y_idx >= y_len) { valid = 0; break; }
            cplx_t term = cplx_mul(y[y_idx], cplx_conj(preamble_up[n]));
            acc = cplx_add(acc, term);
        }
        if (!valid) continue;

        float mag = cplx_abs(acc);
        if (mag > best_peak) {
            best_peak = mag;
            best_lag = lag;
        }
    }

    *fine_peak_out = best_peak;
    return best_lag;
}

// Ham chinh: thay the hoan toan cross_correlate_range cu, dung 2 tang coarse+fine.
// pss_raw: chuoi ZC GOC (chua oversample, NZC mau) - dung cho tang coarse
// preamble_up: chuoi ZC DA oversample qua RRC (dung cho tang fine, giu nguyen
//              nhu preamble_bb_ideal trong MATLAB)
void frame_sync_two_stage(const cplx_t *y, int y_len,
                            const cplx_t *pss_raw, int nzc, int L,
                            const cplx_t *preamble_up, int preamble_up_len,
                            int lag_min, int lag_max,
                            int *best_lag, float *peak_val,
                            float *noise_floor_out) {

    float coarse_peak;
    int coarse_lag = coarse_search(y, y_len, pss_raw, nzc, L,
                                     lag_min, lag_max, &coarse_peak);

    float fine_peak;
    int fine_lag = fine_search(y, y_len, preamble_up, preamble_up_len,
                                 coarse_lag, L, &fine_peak);

    *best_lag = fine_lag;
    *peak_val = fine_peak;

    // Uoc luong noise floor: lay mau xc tai vai trieu diem CACH XA dinh
    // (khong lam full scan, chi lay mau thua de xap xi median - du chinh xac
    //  cho muc dich kiem tra Peak/Noise ratio)
    #define NOISE_SAMPLE_COUNT 200
    float noise_samples[NOISE_SAMPLE_COUNT];
    int step = (lag_max - lag_min) / NOISE_SAMPLE_COUNT;
    if (step < 1) step = 1;

    int cnt = 0;
    for (int lag = lag_min; lag <= lag_max && cnt < NOISE_SAMPLE_COUNT; lag += step) {
        // Bo qua vung gan dinh that (tranh lay chinh dinh lam "noise")
        if (abs(lag - fine_lag) < L*2) continue;

        cplx_t acc = cplx_make(0.0f, 0.0f);
        int valid = 1;
        for (int k = 0; k < nzc; k++) {
            int y_idx = lag + k*L;
            if (y_idx < 0 || y_idx >= y_len) { valid = 0; break; }
            cplx_t term = cplx_mul(y[y_idx], cplx_conj(pss_raw[k]));
            acc = cplx_add(acc, term);
        }
        if (!valid) continue;
        noise_samples[cnt++] = cplx_abs(acc);
    }

    if (cnt > 0) {
        qsort(noise_samples, cnt, sizeof(float), cmp_float);
        *noise_floor_out = noise_samples[cnt/2];
    } else {
        *noise_floor_out = 1e-6f;  // fallback tranh chia 0
    }
}

// Giu lai ham cu (brute-force) cho truong hop can do chinh xac tuyet doi tren
// mot khoang RAT NHO (vi du debug/kiem chung ket qua tang fine) - KHONG dung
// cho search tren toan bo buffer.
void cross_correlate_range(const cplx_t *y, int y_len,
                             const cplx_t *preamble, int preamble_len,
                             int lag_min, int lag_max,
                             int *best_lag, float *peak_val,
                             float *noise_floor_out) {
    int range_len = lag_max - lag_min + 1;
    float *xc_abs = (float*)malloc(range_len * sizeof(float));

    float local_peak = -1.0f;
    int local_best_lag = lag_min;

    for (int lag = lag_min; lag <= lag_max; lag++) {
        cplx_t acc = cplx_make(0.0f, 0.0f);
        for (int n = 0; n < preamble_len; n++) {
            int y_idx = lag + n;
            if (y_idx < 0 || y_idx >= y_len) continue;
            cplx_t term = cplx_mul(y[y_idx], cplx_conj(preamble[n]));
            acc = cplx_add(acc, term);
        }
        float mag = cplx_abs(acc);
        xc_abs[lag - lag_min] = mag;

        if (mag > local_peak) {
            local_peak = mag;
            local_best_lag = lag;
        }
    }

    *best_lag  = local_best_lag;
    *peak_val  = local_peak;

    float *tmp = (float*)malloc(range_len * sizeof(float));
    memcpy(tmp, xc_abs, range_len * sizeof(float));
    qsort(tmp, range_len, sizeof(float), cmp_float);
    *noise_floor_out = tmp[range_len/2];

    free(xc_abs);
    free(tmp);
}
