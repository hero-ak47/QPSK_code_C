#ifndef RLS_EQUALIZER_H
#define RLS_EQUALIZER_H

#include "complex_math.h"
#include <stddef.h>

void rls_equalize_zc(const cplx_t *rx_payload_cfo, int payload_len,
                       const int *pilot_indices, int n_pilots,
                       const cplx_t *zc_pilots,
                       const int *data_indices, int n_data,
                       cplx_t *out_eq, cplx_t *out_data_eq,
                       float lambda_warmup, float lambda_main,
                       float P_init, cplx_t W_init, int n_warmup);

#endif // RLS_EQUALIZER_H
