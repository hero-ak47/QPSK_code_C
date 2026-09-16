#ifndef DOPPLER_COMPENSATE_H
#define DOPPLER_COMPENSATE_H

#include "complex_math.h"

// Uoc luong he so Doppler (time-scale) bang 2 preamble (dau + cuoi frame),
// tuong duong doan "2-PILOT DOPPLER ESTIMATION" trong MATLAB.
//
// y: tin hieu baseband complex (thay cho y_mf trong MATLAB - FPGA da lam RCC filter)
// y_len: do dai y
// pss_raw, nzc, L: chuoi ZC goc va tham so oversample (dung cho coarse search)
// preamble_up, preamble_up_len: preamble da oversample (dung cho fine search)
// search_limit_lag: gioi han tim kiem cho dinh 1 (nua dau buffer)
// N_between_symbols: khoang cach danh dinh (so symbol) giua dinh 1 va dinh 2
// fs: tan so lay mau
//
// Tra ve: a_hat (he so Doppler uoc luong), va ghi lag1/lag2/peak vao con tro out
float estimate_doppler_2pilot(const cplx_t *y, int y_len,
                                const cplx_t *pss_raw, int nzc, int L,
                                const cplx_t *preamble_up, int preamble_up_len,
                                int search_limit_lag, int N_between_symbols, float fs,
                                int *lag1_out, int *lag2_out,
                                float *peak1_out, float *peak2_out);

// Bu Doppler bang resample tuyen tinh (xap xi cho interp1(...,'spline') cua MATLAB -
// spline chinh xac hon nhung ton nhieu tai nguyen embedded hon nhieu; linear la lua
// chon thuc te cho realtime tren Zynq PS, sai so thuong nho neu a_hat nho nhu ky vong)
//
// y_in: tin hieu goc, y_in_len: do dai
// a_hat: he so Doppler da uoc luong
// fs: tan so lay mau
// y_out: buffer output, PHAI duoc cap phat truoc voi kich thuoc >= y_in_len
// Tra ve: so mau thuc te ghi vao y_out (== y_in_len, vi ta giu nguyen do dai va
//          chi noi suy lai gia tri tai cac thoi diem da "co dan")
int doppler_resample_linear(const cplx_t *y_in, int y_in_len,
                              float a_hat, float fs,
                              cplx_t *y_out);

#endif // DOPPLER_COMPENSATE_H
