#ifndef FRAME_SYNC_H
#define FRAME_SYNC_H

#include "complex_math.h"

// Ham chinh nen dung: tim kiem 2 tang (coarse-to-fine), nhanh hon hang nghin lan
// so voi vet can, phu hop chay tren embedded ARM (Zynq PS) trong thoi gian thuc.
void frame_sync_two_stage(const cplx_t *y, int y_len,
                            const cplx_t *pss_raw, int nzc, int L,
                            const cplx_t *preamble_up, int preamble_up_len,
                            int lag_min, int lag_max,
                            int *best_lag, float *peak_val,
                            float *noise_floor_out);

// Ham cu (vet can) - CHI dung cho khoang tim kiem RAT NHO (debug/kiem chung),
// KHONG dung cho search tren toan bo buffer vi qua cham.
void cross_correlate_range(const cplx_t *y, int y_len,
                             const cplx_t *preamble, int preamble_len,
                             int lag_min, int lag_max,
                             int *best_lag, float *peak_val,
                             float *noise_floor_out);

#endif // FRAME_SYNC_H
