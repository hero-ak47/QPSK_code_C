#ifndef DEMOD_DEINTERLEAVE_H
#define DEMOD_DEINTERLEAVE_H

#include "complex_math.h"

void qpsk_demod(const cplx_t *rx_data_eq, int n_symbols, unsigned char *out_bits);
void deinterleave_bits(const unsigned char *bits_il, const int *pattern,
                         int n_bits, unsigned char *bits_out);

#endif // DEMOD_DEINTERLEAVE_H
