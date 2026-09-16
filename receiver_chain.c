// ============================================================================
// receiver_chain.c
//
// FULL DEBUG VERSION - CO THE BAT/TAT TUNG KHOI
//
//   I/Q input
//   -> ZC frame synchronization
//   -> Doppler estimation + compensation     [ENABLE_DOPPLER]
//   -> frame synchronization again           [neu Doppler ON]
//   -> symbol extraction
//   -> payload extraction
//   -> CFO estimation + correction            [ENABLE_CFO]
//   -> RLS channel equalization               [ENABLE_RLS]
//   -> residual phase correction               [ENABLE_PHASE]
//   -> QPSK demodulation
//   -> deinterleaver
//   -> Viterbi decode
//
// System:
//   Fs = 48000 Hz
//   Fc = 14000 Hz
//   Rb = 400 bps
//   Rs = 200 baud
//   L  = 240 samples/symbol
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "receiver_chain.h"
#include "complex_math.h"
#include "config.h"
#include "preamble_gen.h"
#include "rrc_filter.h"
#include "frame_sync.h"
#include "doppler_compensate.h"
#include "cfo_estimate.h"
#include "rls_equalizer.h"
#include "demod_deinterleave.h"
#include "viterbi_decoder.h"

// ============================================================================
// INTERLEAVER
// ============================================================================

#if __has_include("interleaver_pattern.h")

#include "interleaver_pattern.h"

#else

#error "Khong tim thay interleaver_pattern.h"

#endif

// ============================================================================
// DEBUG SWITCH
// ============================================================================
//
// 0 = OFF
// 1 = ON
//
// De test theo thu tu:
//   0000 : baseline
//   0100 : CFO
//   0110 : CFO + RLS
//   0111 : CFO + RLS + PHASE
//   1111 : FULL
//
// ============================================================================

#define ENABLE_DOPPLER 1
#define ENABLE_CFO 1
#define ENABLE_RLS 1
#define ENABLE_PHASE 1

// Timing offset.
// Baseline capture hien tai da tung cho BER = 0 voi offset = 0.
// Neu muon test chain cu cua MATLAB/C thi co the doi thanh 720.
//
// ============================================================================

#define RX_SYMBOL_OFFSET 0

// ============================================================================
// LOAD I/Q INT32 -> COMPLEX FLOAT
// ============================================================================

static void load_iq_from_dma_buffer(
    const int32_t *dma_buf,
    int n_samples,
    cplx_t *y_out,
    float scale)
{
    for (int i = 0; i < n_samples; i++)
    {
        float I =
            (float)dma_buf[2 * i] * scale;

        float Q =
            (float)dma_buf[2 * i + 1] * scale;

        y_out[i] =
            cplx_make(I, Q);
    }
}

// ============================================================================
// BIT -> TEXT (ASCII)
// ============================================================================

static void bits_to_text(
    const unsigned char *bits,
    int n_bits)
{
    int n_bytes = n_bits / 8;

    printf("\n=== DECODED TEXT ===\n");

    for (int byte_idx = 0;
         byte_idx < n_bytes;
         byte_idx++)
    {
        unsigned char value = 0;

        for (int b = 0; b < 8; b++)
        {
            value =
                (unsigned char)((value << 1) |
                                (bits[byte_idx * 8 + b] & 1));
        }

        // 0x00 = padding -> ket thuc text
        if (value == 0)
            break;

        printf("%c", value);
    }

    printf("\n");
}
// ============================================================================
// MAIN RECEIVER CHAIN
// ============================================================================
int run_receiver_chain(
    const int32_t *dma_buf,
    int n_samples,
    unsigned char *decoded_info_out)
{
    int ret_code = 0;

    cplx_t *y_mf = NULL;
    float *h_rrc = NULL;
    cplx_t *preamble_bb_ideal = NULL;
    cplx_t *y_resampled = NULL;

    // ========================================================================
    // 0. LOAD I/Q
    // ========================================================================

    y_mf =
        (cplx_t *)malloc(
            n_samples * sizeof(cplx_t));

    if (!y_mf)
    {
        printf(
            "ERROR: malloc y_mf that bai\n");

        ret_code = -1;
        goto cleanup_error;
    }

    {
        const float scale =
            1.0f / 2147483648.0f;

        load_iq_from_dma_buffer(
            dma_buf,
            n_samples,
            y_mf,
            scale);
    }

    printf(
        "Da doc %d mau I/Q (~%.2f giay)\n",
        n_samples,
        (float)n_samples / FS);

    // ========================================================================
    // PRINT DEBUG MODE
    // ========================================================================

    printf("\n");
    printf("====================================================\n");
    printf("DEBUG CONFIGURATION\n");
    printf("DOPPLER : %s\n",
           ENABLE_DOPPLER ? "ON" : "OFF");
    printf("CFO     : %s\n",
           ENABLE_CFO ? "ON" : "OFF");
    printf("RLS     : %s\n",
           ENABLE_RLS ? "ON" : "OFF");
    printf("PHASE   : %s\n",
           ENABLE_PHASE ? "ON" : "OFF");
    printf("TIMING OFFSET = %d samples\n",
           RX_SYMBOL_OFFSET);
    printf("====================================================\n");

    // ========================================================================
    // 1. TAO PREAMBLE ZC + RRC
    // ========================================================================

    cplx_t pss[NZC];

    generate_zc_sequence(
        pss,
        NZC,
        ZC_ROOT_U);

    int rrc_len =
        RRC_SPAN * L_SPS + 1;

    h_rrc =
        (float *)malloc(
            rrc_len * sizeof(float));

    if (!h_rrc)
    {
        printf(
            "ERROR: malloc h_rrc that bai\n");

        ret_code = -1;
        goto cleanup_error;
    }

    rrc_design(
        h_rrc,
        RRC_SPAN,
        L_SPS,
        RRC_BETA);

    int preamble_up_len =
        NZC * L_SPS +
        rrc_len - 1;

    preamble_bb_ideal =
        (cplx_t *)malloc(
            preamble_up_len *
            sizeof(cplx_t));

    if (!preamble_bb_ideal)
    {
        printf(
            "ERROR: malloc preamble_bb_ideal that bai\n");

        ret_code = -1;
        goto cleanup_error;
    }

    int actual_preamble_len =
        upfirdn_upsample(
            pss,
            NZC,
            h_rrc,
            rrc_len,
            L_SPS,
            preamble_bb_ideal);

    printf(
        "Da tao preamble mau, do dai = %d mau\n",
        actual_preamble_len);

    // ========================================================================
    // 2. FRAME SYNC LAN 1
    // ========================================================================

    int search_limit_lag =
        n_samples / 2;

    int best_lag1 = 0;

    float peak1_val = 0.0f;
    float noise_floor = 0.0f;

    frame_sync_two_stage(
        y_mf,
        n_samples,

        pss,
        NZC,
        L_SPS,

        preamble_bb_ideal,
        actual_preamble_len,

        0,
        search_limit_lag,

        &best_lag1,
        &peak1_val,
        &noise_floor);

    int start_sample =
        best_lag1;

    /* TEST: canh start_sample ve vi tri tuong ung MATLAB */
    start_sample -= 476;

    if (start_sample < 0)
        start_sample = 0;

    printf("\n");
    printf("--- FRAME SYNC LAN 1 ---\n");

    printf(
        "best_lag1 = %d\n",
        best_lag1);

    printf(
        "start_sample sau offset test = %d\n",
        start_sample);

    printf(
        "Peak1/Noise = %.2f\n",
        peak1_val /
            (noise_floor + 1e-9f));

    if (peak1_val /
            (noise_floor + 1e-9f) <
        5.0f)
    {
        printf(
            "CANH BAO: Peak1/Noise thap!\n");
    }

    if (start_sample < 0 ||
        (start_sample +
         N_BETWEEN_SYMBOLS * L_SPS) >
            n_samples)
    {
        printf(
            "ERROR: Khong tim thay frame!\n");

        ret_code = -1;
        goto cleanup_error;
    }
    // ========================================================================
    // 3. DOPPLER
    // ========================================================================

#if ENABLE_DOPPLER

    {
        int lag1_dop = 0;
        int lag2_dop = 0;

        float peak1_dop = 0.0f;
        float peak2_dop = 0.0f;

        float a_hat =
            estimate_doppler_2pilot(
                y_mf,
                n_samples,

                pss,
                NZC,
                L_SPS,

                preamble_bb_ideal,
                actual_preamble_len,

                search_limit_lag,
                N_BETWEEN_SYMBOLS,
                (float)FS,

                &lag1_dop,
                &lag2_dop,
                &peak1_dop,
                &peak2_dop);

        printf("\n");
        printf("--- DOPPLER ---\n");

        printf(
            "T0 = %.6f s\n",
            (float)N_BETWEEN_SYMBOLS *
                (float)L_SPS /
                (float)FS);

        printf(
            "Dinh 1: lag = %d, bien do = %.3f\n",
            lag1_dop,
            peak1_dop);

        printf(
            "Dinh 2: lag = %d, bien do = %.3f\n",
            lag2_dop,
            peak2_dop);

        printf(
            "a_hat = %.9f\n",
            a_hat);

        printf(
            "f_D ~ %.3f Hz\n",
            a_hat * (float)FC);

        // ------------------------------------------------------------
        // Doppler resample
        // ------------------------------------------------------------

        y_resampled =
            (cplx_t *)malloc(
                n_samples *
                sizeof(cplx_t));

        if (!y_resampled)
        {
            printf(
                "ERROR: malloc y_resampled that bai\n");

            ret_code = -1;
            goto cleanup_error;
        }

        doppler_resample_linear(
            y_mf,
            n_samples,
            a_hat,
            (float)FS,
            y_resampled);

        printf(
            "Da bu Doppler time-scale\n");

        // ------------------------------------------------------------
        // Frame sync lan 2
        // ------------------------------------------------------------

        int best_lag2 = 0;

        float peak2_val = 0.0f;
        float noise_floor2 = 0.0f;

        frame_sync_two_stage(
            y_resampled,
            n_samples,

            pss,
            NZC,
            L_SPS,

            preamble_bb_ideal,
            actual_preamble_len,

            0,
            search_limit_lag,

            &best_lag2,
            &peak2_val,
            &noise_floor2);

        printf("\n");
        printf(
            "--- FRAME SYNC LAN 2 SAU DOPPLER ---\n");

        printf(
            "start_sample moi = %d\n",
            best_lag2);

        printf(
            "Peak1/Noise = %.2f\n",
            peak2_val /
                (noise_floor2 + 1e-9f));

        if (best_lag2 < 0 ||
            (best_lag2 +
             N_BETWEEN_SYMBOLS * L_SPS) >
                n_samples)
        {
            printf(
                "ERROR: Frame sau Doppler loi!\n");

            ret_code = -1;
            goto cleanup_error;
        }

        start_sample =
            best_lag2;

        // ------------------------------------------------------------
        // Dung signal sau resample
        // ------------------------------------------------------------

        free(y_mf);

        y_mf =
            y_resampled;

        y_resampled =
            NULL;
    }

#else

    printf("\n");
    printf("--- DOPPLER ---\n");
    printf("OFF -> bo qua Doppler estimation/resampling\n");

#endif

    // ========================================================================
    // 4. SYMBOL EXTRACTION
    // ========================================================================

    cplx_t rx_symbols[TOTAL_SYM_LEN];

    int first_sym_offset =
        RX_SYMBOL_OFFSET;

    printf("\n");
    printf("--- SYMBOL EXTRACTION ---\n");

    printf(
        "start_sample     = %d\n",
        start_sample);

    printf(
        "first_sym_offset = %d\n",
        first_sym_offset);

    printf(
        "L_SPS            = %d\n",
        L_SPS);

    for (int k = 0;
         k < TOTAL_SYM_LEN;
         k++)
    {
        int idx =
            start_sample +
            first_sym_offset +
            k * L_SPS;

        if (idx < 0 ||
            idx >= n_samples)
        {
            printf(
                "ERROR: symbol index vuot bien "
                "(k=%d, idx=%d)\n",
                k,
                idx);

            ret_code = -1;
            goto cleanup_error;
        }

        rx_symbols[k] =
            y_mf[idx];
    }

    // ========================================================================
    // 5. PAYLOAD
    // ========================================================================

    int payload_len =
        TOTAL_SYM_LEN -
        SKIP_SYM -
        NZC;

    if (payload_len != N_PAYLOAD)
    {
        printf(
            "ERROR: payload_len = %d, "
            "N_PAYLOAD = %d\n",
            payload_len,
            N_PAYLOAD);

        ret_code = -1;
        goto cleanup_error;
    }

    cplx_t rx_payload[N_PAYLOAD];

    for (int i = 0;
         i < payload_len;
         i++)
    {
        rx_payload[i] =
            rx_symbols[SKIP_SYM + i];
    }

    printf(
        "Da trich xuat %d symbol payload\n",
        payload_len);

    // ========================================================================
    // 6. PILOT / DATA INDICES
    // ========================================================================

    int pilot_indices[N_PILOTS];
    int data_indices[N_DATA];

    int n_pilots_actual = 0;
    int n_data_actual = 0;

    for (int i = 0;
         i < N_PAYLOAD;
         i++)
    {
        if ((i % PILOT_STEP) == 0)
        {
            if (n_pilots_actual <
                N_PILOTS)
            {
                pilot_indices[n_pilots_actual++] =
                    i;
            }
        }
        else
        {
            if (n_data_actual <
                N_DATA)
            {
                data_indices[n_data_actual++] =
                    i;
            }
        }
    }

    printf(
        "Pilot = %d symbol\n",
        n_pilots_actual);

    printf(
        "Data  = %d symbol\n",
        n_data_actual);

    if (n_pilots_actual != N_PILOTS ||
        n_data_actual != N_DATA)
    {
        printf(
            "ERROR: pilot/data count sai!\n");

        ret_code = -1;
        goto cleanup_error;
    }

    // ========================================================================
    // 7. ZC PILOTS
    // ========================================================================

    cplx_t zc_pilots[N_PILOTS];

    generate_zc_pilots(
        zc_pilots,
        n_pilots_actual,
        ZC_ROOT_U_PILOT);

    printf("\n");
    printf("--- ZC PILOT ---\n");

    printf(
        "ZC pilot root = %d\n",
        ZC_ROOT_U_PILOT);

    // ========================================================================
    // 8. CFO
    // ========================================================================
    //
    // QUAN TRONG:
    // estimate_and_correct_cfo() cua project
    // sua truc tiep rx_payload.
    //
    // Khi OFF -> khong sua rx_payload.
    //
    // ========================================================================

#if ENABLE_CFO

    {
        float cfo_est =
            estimate_and_correct_cfo(
                rx_payload,
                payload_len,

                pilot_indices,
                n_pilots_actual,

                zc_pilots,

                L_SPS,
                (float)FS);

        printf("\n");
        printf("--- CFO ---\n");

        printf(
            "CFO = %.6f Hz\n",
            cfo_est);

        printf(
            "CFO correction = ON\n");
    }

#else

    printf("\n");
    printf("--- CFO ---\n");
    printf("OFF -> khong uoc luong/khong bu CFO\n");

#endif

    // ========================================================================
    // 9. RLS
    // ========================================================================

    cplx_t rx_payload_eq[N_PAYLOAD];
    cplx_t rx_data_eq[N_DATA];

#if ENABLE_RLS

    {
        rls_equalize_zc(
            rx_payload,
            payload_len,

            pilot_indices,
            n_pilots_actual,

            zc_pilots,

            data_indices,
            n_data_actual,

            rx_payload_eq,
            rx_data_eq,

            0.2f,   // lambda_warmup
            0.85f,  // lambda_main
            100.0f, // P_init
            cplx_make(
                1.0f,
                0.0f), // W_init
            50);       // N_warmup

        printf("\n");
        printf("--- RLS ---\n");
        printf(
            "ON -> Da can bang RLS\n");
    }

#else

    {
        /*
         * RLS OFF:
         * lay thang data tu payload.
         */

        for (int i = 0;
             i < n_data_actual;
             i++)
        {
            rx_data_eq[i] =
                rx_payload[data_indices[i]];
        }

        for (int i = 0;
             i < payload_len;
             i++)
        {
            rx_payload_eq[i] =
                rx_payload[i];
        }

        printf("\n");
        printf("--- RLS ---\n");
        printf(
            "OFF -> bypass RLS\n");
    }

#endif

    // ========================================================================
    // 10. DEBUG DATA SYMBOL
    // ========================================================================

    printf("\n");
    printf(
        "--- RX DATA SYMBOL TRUOC PHASE ---\n");

    for (int i = 0;
         i < 10 &&
         i < n_data_actual;
         i++)
    {
        printf(
            "%3d : %+.6f %+.6fj\n",
            i,
            rx_data_eq[i].re,
            rx_data_eq[i].im);
    }

    // ========================================================================
    // 11. RESIDUAL PHASE
    // ========================================================================

#if ENABLE_PHASE

    {
        /*
         * QPSK ideal:
         *
         * +45 deg
         * +135 deg
         * -135 deg
         * -45 deg
         */

        int n_check =
            (n_data_actual < 400)
                ? n_data_actual
                : 400;

        const float constellation_angles[4] =
            {
                (float)M_PI / 4.0f,
                3.0f * (float)M_PI / 4.0f,
                -3.0f * (float)M_PI / 4.0f,
                -(float)M_PI / 4.0f};

        float sum_cos = 0.0f;
        float sum_sin = 0.0f;

        for (int i = 0;
             i < n_check;
             i++)
        {
            float rx_phase =
                cplx_angle(
                    rx_data_eq[i]);

            float best_err = 1e9f;

            for (int c = 0;
                 c < 4;
                 c++)
            {
                float diff =
                    rx_phase -
                    constellation_angles[c];

                while (diff > M_PI)
                {
                    diff -=
                        2.0f *
                        (float)M_PI;
                }

                while (diff < -M_PI)
                {
                    diff +=
                        2.0f *
                        (float)M_PI;
                }

                if (fabsf(diff) <
                    fabsf(best_err))
                {
                    best_err =
                        diff;
                }
            }

            sum_cos +=
                cosf(best_err);

            sum_sin +=
                sinf(best_err);
        }

        float phi =
            atan2f(
                sum_sin,
                sum_cos);

        printf("\n");
        printf(
            "--- RESIDUAL PHASE ---\n");

        printf(
            "phi = %.9f rad "
            "(%.6f deg)\n",
            phi,
            phi *
                180.0f /
                (float)M_PI);

        cplx_t corr =
            cplx_expj(-phi);

        for (int i = 0;
             i < n_data_actual;
             i++)
        {
            rx_data_eq[i] =
                cplx_mul(
                    rx_data_eq[i],
                    corr);
        }

        printf(
            "Residual phase correction = ON\n");
    }

#else

    printf("\n");
    printf("--- RESIDUAL PHASE ---\n");
    printf(
        "OFF -> bypass residual phase\n");

#endif

    // ========================================================================
    // 12. DEBUG SYMBOL SAU TOAN BO BUOC CAN BANG
    // ========================================================================

    printf("\n");
    printf(
        "--- RX DATA SYMBOL CUOI ---\n");

    for (int i = 0;
         i < 10 &&
         i < n_data_actual;
         i++)
    {
        printf(
            "%3d : %+.6f %+.6fj\n",
            i,
            rx_data_eq[i].re,
            rx_data_eq[i].im);
    }

    // ========================================================================
    // 13. QPSK DEMOD
    // ========================================================================

    unsigned char
        rx_coded_bits_il[N_CODED_BITS];

    qpsk_demod(
        rx_data_eq,
        n_data_actual,
        rx_coded_bits_il);

    printf("\n");
    printf(
        "--- QPSK DEMOD ---\n");

    printf(
        "RX coded IL 20 bit dau: ");

    for (int i = 0;
         i < 20 &&
         i < N_CODED_BITS;
         i++)
    {
        printf(
            "%d ",
            rx_coded_bits_il[i]);
    }

    printf("\n");

    // ========================================================================
    // 14. SO SANH IL
    // ========================================================================

    FILE *f_il =
        fopen(
            "coded_bits_il.bin",
            "rb");

    if (!f_il)
    {
        printf(
            "WARNING: khong mo duoc "
            "coded_bits_il.bin\n");
    }
    else
    {
        unsigned char
            ref_il[N_CODED_BITS];

        size_t nr =
            fread(
                ref_il,
                1,
                N_CODED_BITS,
                f_il);

        fclose(f_il);

        if (nr != N_CODED_BITS)
        {
            printf(
                "WARNING: coded_bits_il.bin "
                "co %zu bit\n",
                nr);
        }
        else
        {
            int err_il = 0;

            for (int i = 0;
                 i < N_CODED_BITS;
                 i++)
            {
                if (rx_coded_bits_il[i] !=
                    ref_il[i])
                {
                    err_il++;
                }
            }

            printf("\n");
            printf(
                "=== SO SANH TRUOC DEINTERLEAVE ===\n");

            printf(
                "IL errors = %d / %d\n",
                err_il,
                N_CODED_BITS);

            printf(
                "IL BER = %.8f\n",
                (float)err_il /
                    (float)N_CODED_BITS);
        }
    }

    // ========================================================================
    // 15. DE-INTERLEAVER
    // ========================================================================

    unsigned char
        rx_coded_bits[N_CODED_BITS];

    deinterleave_bits(
        rx_coded_bits_il,
        interleaver_pattern,
        N_CODED_BITS,
        rx_coded_bits);

    printf("\n");
    printf(
        "--- DE-INTERLEAVER ---\n");

    printf(
        "RX coded 20 bit dau: ");

    for (int i = 0;
         i < 20 &&
         i < N_CODED_BITS;
         i++)
    {
        printf(
            "%d ",
            rx_coded_bits[i]);
    }

    printf("\n");

    // ========================================================================
    // 16. SO SANH CODED
    // ========================================================================

    FILE *f_coded =
        fopen(
            "coded_bits.bin",
            "rb");

    if (!f_coded)
    {
        printf(
            "WARNING: khong mo duoc "
            "coded_bits.bin\n");
    }
    else
    {
        unsigned char
            ref_coded[N_CODED_BITS];

        size_t nr =
            fread(
                ref_coded,
                1,
                N_CODED_BITS,
                f_coded);

        fclose(f_coded);

        if (nr != N_CODED_BITS)
        {
            printf(
                "WARNING: coded_bits.bin "
                "co %zu bit\n",
                nr);
        }
        else
        {
            int err_coded = 0;

            for (int i = 0;
                 i < N_CODED_BITS;
                 i++)
            {
                if (rx_coded_bits[i] !=
                    ref_coded[i])
                {
                    err_coded++;
                }
            }

            printf("\n");
            printf(
                "=== SO SANH SAU DEINTERLEAVE ===\n");

            printf(
                "Coded errors = %d / %d\n",
                err_coded,
                N_CODED_BITS);

            printf(
                "Coded BER = %.8f\n",
                (float)err_coded /
                    (float)N_CODED_BITS);
        }
    }

    // ========================================================================
    // 17. VITERBI
    // ========================================================================

    printf("\n");
    printf(
        "--- VITERBI ---\n");

    viterbi_build_trellis(
        GEN_POLY_1,
        GEN_POLY_2,
        CONSTRAINT_LEN);

    int n_time_steps =
        N_INFO + N_TAIL;

    unsigned char
        decoded_bits[N_INFO + N_TAIL];

    viterbi_decode_hard(
        rx_coded_bits,
        n_time_steps,
        decoded_bits);

    memcpy(
        decoded_info_out,
        decoded_bits,
        N_INFO *
            sizeof(unsigned char));

    printf(
        "Decoded %d bit thong tin\n",
        N_INFO);

    printf(
        "10 bit dau: ");

    for (int i = 0;
         i < 10 &&
         i < N_INFO;
         i++)
    {
        printf(
            "%d ",
            decoded_info_out[i]);
    }

    printf("\n");

    // ========================================================================
    // BIT -> TEXT
    // ========================================================================

    bits_to_text(
        decoded_info_out,
        N_INFO);

    viterbi_free_trellis();

    // ========================================================================
    // 18. BER
    // ========================================================================

    FILE *f_info =
        fopen(
            "info_bits.bin",
            "rb");

    if (!f_info)
    {
        printf(
            "WARNING: khong mo duoc "
            "info_bits.bin\n");
    }
    else
    {
        unsigned char
            ref_info[N_INFO];

        size_t nr =
            fread(
                ref_info,
                1,
                N_INFO,
                f_info);

        fclose(f_info);

        if (nr != N_INFO)
        {
            printf(
                "WARNING: info_bits.bin "
                "co %zu bit\n",
                nr);
        }
        else
        {
            int err_info = 0;

            for (int i = 0;
                 i < N_INFO;
                 i++)
            {
                if (decoded_info_out[i] !=
                    ref_info[i])
                {
                    err_info++;
                }
            }

            printf("\n");
            printf(
                "=== BER ===\n");

            printf(
                "So bit thong tin = %d\n",
                N_INFO);

            printf(
                "So bit loi       = %d\n",
                err_info);

            printf(
                "BER              = %.8f\n",
                (float)err_info /
                    (float)N_INFO);
        }
    }

    // ========================================================================
    // DONE
    // ========================================================================

    viterbi_free_trellis();

    ret_code = 0;

cleanup_error:

    free(y_mf);
    free(h_rrc);
    free(preamble_bb_ideal);
    free(y_resampled);

    return ret_code;
}