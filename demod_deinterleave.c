#include "demod_deinterleave.h"

// Port cua buoc 7: giai dieu che QPSK (demapping) - ra bit da ma hoa & da interleave
// rx_data_eq[i] -> 2 bit: bit(2i) = real<0, bit(2i+1) = imag<0
// out phai co kich thuoc >= 2*n_symbols
void qpsk_demod(const cplx_t *rx_data_eq, int n_symbols, unsigned char *out_bits) {
    for (int i = 0; i < n_symbols; i++) {
        out_bits[2*i]     = (rx_data_eq[i].re < 0.0f) ? 1 : 0;
        out_bits[2*i + 1] = (rx_data_eq[i].im < 0.0f) ? 1 : 0;
    }
}

// Port cua deinterleaver: rx_coded_bits(interleaver_pattern) = rx_coded_bits_il
// Nghia la: bit tai vi tri interleaver_pattern[k] (trong mang goc) = rx_coded_bits_il[k]
// pattern da duoc chuyen sang 0-based khi tao file interleaver_pattern.h
void deinterleave_bits(const unsigned char *bits_il, const int *pattern,
                         int n_bits, unsigned char *bits_out) {
    for (int k = 0; k < n_bits; k++) {
        bits_out[pattern[k]] = bits_il[k];
    }
}
