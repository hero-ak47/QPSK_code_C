#ifndef RRC_FILTER_H
#define RRC_FILTER_H

#include "complex_math.h"

void rrc_design(float *h, int span, int L, float beta);
int  upfirdn_upsample(const cplx_t *in, int in_len, const float *h, int h_len,
                        int L, cplx_t *out);
void fir_filter_same(const cplx_t *in, int in_len, const float *h, int h_len,
                       cplx_t *out);

#endif // RRC_FILTER_H
