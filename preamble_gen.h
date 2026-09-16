#ifndef PREAMBLE_GEN_H
#define PREAMBLE_GEN_H

#include "complex_math.h"

void generate_zc_sequence(cplx_t *out, int N, int u);
void generate_zc_pilots(cplx_t *out, int N, int u);

#endif // PREAMBLE_GEN_H
