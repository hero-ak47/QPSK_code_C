#ifndef CFO_ESTIMATE_H
#define CFO_ESTIMATE_H

#include "complex_math.h"

float estimate_and_correct_cfo(cplx_t *rx_payload, int payload_len,
                                 const int *pilot_indices, int n_pilots,
                                 const cplx_t *zc_pilots,
                                 int L, float fs);

#endif // CFO_ESTIMATE_H
