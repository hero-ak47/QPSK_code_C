#ifndef CONFIG_H
#define CONFIG_H

#include "complex_math.h"
#include "interleaver_pattern.h"

/* ============================================================
 * THONG SO HE THONG
 * KHOA THEO MATLAB TX:
 *   Fs = 48000 Hz
 *   Fc = 14000 Hz
 *   Rb = 400 bps
 *   Rs = 200 baud
 *   L  = 240 samples/symbol
 * ============================================================ */

#define FS 48000
#define FC 14000

#define RS 200
#define L_SPS (FS / RS) /* 240 */

#define NZC 63
#define ZC_ROOT_U 25
#define ZC_ROOT_U_PILOT 7

#define N_GUARD 14
#define ZC_ROOT_U_GUARD 41

#define N_PAYLOAD 500
#define N_BETWEEN_SYMBOLS (NZC + N_GUARD + N_PAYLOAD)

/* Payload:
 * pilot, data, data, data, data
 * lap lai 100 lan
 */
#define PILOT_STEP 5
#define N_PILOTS 100
#define N_DATA 400

#define NZC 63
#define ZC_ROOT_U 25
#define ZC_ROOT_PILOT 7
#define ZC_ROOT_FOOTER 41
/* Tong frame symbol:
 * 63 preamble
 * 14 guard
 * 500 payload
 * 63 footer/preamble cuoi
 */
#define TOTAL_SYM_LEN (NZC + N_GUARD + N_PAYLOAD + NZC) /* 640 */
#define SKIP_SYM (NZC + N_GUARD)                        /* 77 */

#define RRC_SPAN 6
#define RRC_BETA 0.5f

#define ENABLE_DOPPLER_COMP 0

/* ============================================================
 * CHANNEL CODING
 * MATLAB:
 *   constraint_length = 7
 *   code_generators = [171 133]  % OCTAL
 *   n_tail = 6
 *   n_info = 394
 *   coded_bits = 800
 * ============================================================ */

#if __has_include("config_generated.h")

#include "config_generated.h"

#define CONSTRAINT_LEN CONSTRAINT_LEN_GEN
#define GEN_POLY_1 GEN_POLY_1_GEN
#define GEN_POLY_2 GEN_POLY_2_GEN
#define N_INFO N_INFO_GEN
#define N_TAIL N_TAIL_GEN
#define N_CODED_BITS N_CODED_BITS_GEN

#else

#warning "config_generated.h khong ton tai - dang dung gia tri MAC DINH tu MATLAB TX!"

#define CONSTRAINT_LEN 7

/* MATLAB [171 133] la OCTAL */
#define GEN_POLY_1 0171
#define GEN_POLY_2 0133

#define N_INFO 394
#define N_TAIL 6
#define N_CODED_BITS ((N_INFO + N_TAIL) * 2) /* 800 */

#endif

#define NUM_STATES (1 << (CONSTRAINT_LEN - 1))
#define TRACEBACK_DEPTH (5 * CONSTRAINT_LEN)

/*
 * interleaver_pattern:
 * Da duoc dinh nghia truc tiep trong interleaver_pattern.h
 *
 * KHONG duoc khai bao lai:
 * extern const int interleaver_pattern[N_CODED_BITS];
 *
 * vi interleaver_pattern.h dang dung:
 * static const int interleaver_pattern[800] = {...};
 */

#endif /* CONFIG_H */