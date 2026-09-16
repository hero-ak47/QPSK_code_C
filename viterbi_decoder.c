#include "viterbi_decoder.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include <limits.h>

// ================= CONVOLUTIONAL ENCODER =================
// Thay the ham convenc(info_bits, trellis) cua MATLAB
// Rate 1/2, constraint length K, 2 generator polynomials (octal, dang so nguyen)
// state: K-1 bit shift register, khoi tao = 0
// out_bits phai co kich thuoc >= 2*n_info_bits
void convolutional_encode(const unsigned char *info_bits, int n_bits,
                          int gen1, int gen2, int constraint_len,
                          unsigned char *out_bits)
{
    unsigned int state = 0; // shift register, bit 0 = moi nhat
    unsigned int mask = (1U << constraint_len) - 1;

    for (int i = 0; i < n_bits; i++)
    {
        // Dua bit moi vao MSB cua "cua so" K bit: state_full = (bit_in << (K-1)) | state
        unsigned int state_full = ((unsigned int)info_bits[i] << (constraint_len - 1)) | state;
        state_full &= mask;

        // Tinh parity cho tung generator (XOR cac bit duoc chon boi generator polynomial)
        unsigned int g1 = (unsigned int)gen1;
        unsigned int g2 = (unsigned int)gen2;

        unsigned int and1 = state_full & g1;
        unsigned int and2 = state_full & g2;

        // Dem so bit 1 (parity) - dung popcount don gian
        unsigned char bit1 = 0, bit2 = 0;
        while (and1)
        {
            bit1 ^= (and1 & 1);
            and1 >>= 1;
        }
        while (and2)
        {
            bit2 ^= (and2 & 1);
            and2 >>= 1;
        }

        out_bits[2 * i] = bit1;
        out_bits[2 * i + 1] = bit2;

        // Cap nhat state: dich phai, dua bit moi vao dau
        state = (state_full >> 1);
    }
}

// ================= VITERBI DECODER (Hard-decision, terminated trellis) =================
// Thay the ham vitdec(rx_coded_bits, trellis, tblen, 'term', 'hard')
//
// So luong states = 2^(K-1)
// Tai moi state, tinh next_state va output bit cho input=0 va input=1,
// dung bang tra cuu (lookup table) tinh truoc.

typedef struct
{
    unsigned int next_state[2];   // next_state[input_bit]
    unsigned char out_bits[2][2]; // out_bits[input_bit][0..1] = 2 bit output
} TrellisEntry;

static TrellisEntry *g_trellis = NULL;
static int g_num_states = 0;

// Xay dung bang trellis mot lan (goi truoc khi decode)
void viterbi_build_trellis(int gen1, int gen2, int constraint_len)
{
    g_num_states = 1 << (constraint_len - 1);
    if (g_trellis)
        free(g_trellis);
    g_trellis = (TrellisEntry *)malloc(g_num_states * sizeof(TrellisEntry));

    unsigned int mask = (1U << constraint_len) - 1;

    for (int s = 0; s < g_num_states; s++)
    {
        for (int input_bit = 0; input_bit <= 1; input_bit++)
        {
            unsigned int state_full = ((unsigned int)input_bit << (constraint_len - 1)) | (unsigned int)s;
            state_full &= mask;

            unsigned int and1 = state_full & (unsigned int)gen1;
            unsigned int and2 = state_full & (unsigned int)gen2;

            unsigned char bit1 = 0, bit2 = 0;
            while (and1)
            {
                bit1 ^= (and1 & 1);
                and1 >>= 1;
            }
            while (and2)
            {
                bit2 ^= (and2 & 1);
                and2 >>= 1;
            }

            unsigned int next_s = state_full >> 1;

            g_trellis[s].next_state[input_bit] = next_s;
            g_trellis[s].out_bits[input_bit][0] = bit1;
            g_trellis[s].out_bits[input_bit][1] = bit2;
        }
    }
}

void viterbi_free_trellis(void)
{
    if (g_trellis)
    {
        free(g_trellis);
        g_trellis = NULL;
    }
}

// Hamming distance giua 2 bit-pair
static inline int hamming2(unsigned char a0, unsigned char a1, unsigned char b0, unsigned char b1)
{
    return (a0 != b0) + (a1 != b1);
}

// Giai ma Viterbi hard-decision, terminated trellis (gia dinh encoder ket thuc o state 0,
// giong 'term' mode trong MATLAB - can K-1 bit tail = 0 o cuoi)
//
// received_bits: mang bit da giai dieu che (2*n_info_plus_tail phan tu)
// n_info_plus_tail: so symbol thoi gian (khong tinh x2 cho rate 1/2)
// decoded_bits_out: output, kich thuoc >= n_info_plus_tail
void viterbi_decode_hard(
    const unsigned char *rx_bits,
    int n_input_bits,
    unsigned char *decoded_bits)
{
    const int NSTATE = 64;
    const int INF = 1000000000;

    /*
     * rx_bits có 2 bit / mỗi input bit
     * n_input_bits = 400
     */

    /* ---------------------------------------------------------
     * path_metric[t][state]
     * survivor_state[t][state]
     * survivor_bit[t][state]
     * --------------------------------------------------------- */

    int *metric_prev = malloc(NSTATE * sizeof(int));
    int *metric_next = malloc(NSTATE * sizeof(int));

    unsigned char *survivor_state =
        malloc(n_input_bits * NSTATE * sizeof(unsigned char));

    unsigned char *survivor_bit =
        malloc(n_input_bits * NSTATE * sizeof(unsigned char));

    if (!metric_prev || !metric_next ||
        !survivor_state || !survivor_bit)
    {
        printf("Viterbi malloc failed!\n");
        return;
    }

    /* ---------------- INIT ---------------- */

    for (int s = 0; s < NSTATE; s++)
        metric_prev[s] = INF;

    /* MATLAB vitdec(...,'term') -> start state = 0 */
    metric_prev[0] = 0;

    /* ---------------- TRELLIS ---------------- */

    /*
     * state:
     *   6-bit memory
     *
     * full_state = [input_bit | state]
     *
     * generator:
     *   0171
     *   0133
     */

    for (int t = 0; t < n_input_bits; t++)
    {
        int r0 = rx_bits[2 * t];
        int r1 = rx_bits[2 * t + 1];

        for (int s = 0; s < NSTATE; s++)
            metric_next[s] = INF;

        for (int prev = 0; prev < NSTATE; prev++)
        {
            if (metric_prev[prev] >= INF)
                continue;

            for (int input = 0; input <= 1; input++)
            {
                int full_state = (input << 6) | prev;

                int next_state = full_state >> 1;

                int out0 = __builtin_parity(
                    full_state & 0171);

                int out1 = __builtin_parity(
                    full_state & 0133);

                int bm =
                    (out0 != r0) +
                    (out1 != r1);

                int new_metric =
                    metric_prev[prev] + bm;

                /*
                 * Nếu có nhiều path cùng metric,
                 * giữ path xuất hiện đầu tiên.
                 */
                if (new_metric < metric_next[next_state])
                {
                    metric_next[next_state] = new_metric;

                    survivor_state[t * NSTATE + next_state] = (unsigned char)prev;

                    survivor_bit[t * NSTATE + next_state] = (unsigned char)input;
                }
            }
        }

        int *tmp = metric_prev;
        metric_prev = metric_next;
        metric_next = tmp;
    }

    /* ---------------- TERMINATION ---------------- */

    /*
     * MATLAB 'term':
     * kết thúc tại state 0
     */
    int state = 0;

    printf("Final metric state 0 = %d\n", metric_prev[0]);

    /* ---------------- TRACEBACK ---------------- */

    for (int t = n_input_bits - 1; t >= 0; t--)
    {
        decoded_bits[t] =
            survivor_bit[t * NSTATE + state];

        state =
            survivor_state[t * NSTATE + state];
    }

    free(metric_prev);
    free(metric_next);
    free(survivor_state);
    free(survivor_bit);
}
