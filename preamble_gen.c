#include "preamble_gen.h"
#include "config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Sinh chuoi Zadoff-Chu: pss[n] = exp(-j*pi*u*n*(n+1)/Nzc)
// Tuong duong dong MATLAB: pss = exp(-1j*pi*u*n.*(n+1)/Nzc);
void generate_zc_sequence(cplx_t *out, int N, int u) {
    for (int n = 0; n < N; n++) {
        float phase = -M_PI * (float)u * (float)n * (float)(n+1) / (float)N;
        out[n] = cplx_expj(phase);
    }
}

// Sinh pilot ZC voi he so sqrt(2), tuong duong:
//   zc_pilots = sqrt(2) * exp(-1j*pi*u_p*n_p.*(n_p+1)/N_pilots);
void generate_zc_pilots(cplx_t *out, int N, int u) {
    generate_zc_sequence(out, N, u);
    for (int n = 0; n < N; n++) {
        out[n] = cplx_scale(out[n], sqrtf(2.0f));
    }
}
