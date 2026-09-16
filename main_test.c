#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "config.h"
#include "receiver_chain.h"

#define CAPTURE_SEC 5
#define N_SAMPLES (FS * CAPTURE_SEC)

int main(int argc, char **argv)
{
    const char *capture_file =
        (argc > 1) ? argv[1] : "capture.bin";

    const char *info_file =
        (argc > 2) ? argv[2] : "info_bits.bin";

    /* =========================================================
     * 1. DOC CAPTURE.BIN
     * ========================================================= */
    FILE *fp = fopen(capture_file, "rb");
    if (!fp)
    {
        printf("ERROR: khong mo duoc file %s\n", capture_file);
        return -1;
    }

    int32_t *dma_buf =
        (int32_t *)malloc(N_SAMPLES * 2 * sizeof(int32_t));

    if (!dma_buf)
    {
        printf("ERROR: malloc dma_buf that bai\n");
        fclose(fp);
        return -1;
    }

    size_t n_read =
        fread(dma_buf, sizeof(int32_t), N_SAMPLES * 2, fp);

    fclose(fp);

    printf("TEST_MODE: da doc %zu phan tu int32 tu file %s\n",
           n_read, capture_file);

    if (n_read < (size_t)(N_SAMPLES * 2))
    {
        printf("CANH BAO: file nho hon ky vong!\n");
        free(dma_buf);
        return -1;
    }

    /* =========================================================
     * 2. CHAY RECEIVER
     * ========================================================= */
    unsigned char decoded_info[N_INFO];

    int status =
        run_receiver_chain(dma_buf, N_SAMPLES, decoded_info);

    if (status != 0)
    {
        printf("\nreceiver_chain that bai, status = %d\n", status);
        free(dma_buf);
        return status;
    }

    /* =========================================================
     * 3. DOC INFO_BITS GOC
     * ========================================================= */
    unsigned char info_bits[N_INFO];

    FILE *fi = fopen(info_file, "rb");
    if (!fi)
    {
        printf("ERROR: khong mo duoc %s\n", info_file);
        free(dma_buf);
        return -1;
    }

    size_t n_info_read =
        fread(info_bits, sizeof(unsigned char), N_INFO, fi);

    fclose(fi);

    if (n_info_read != N_INFO)
    {
        printf("ERROR: info_bits.bin chi co %zu bit, can %d\n",
               n_info_read, N_INFO);
        free(dma_buf);
        return -1;
    }

    /* =========================================================
     * 4. TINH BER
     * ========================================================= */
    int errors = 0;

    for (int i = 0; i < N_INFO; i++)
    {
        if (decoded_info[i] != info_bits[i])
        {
            errors++;
        }
    }

    float ber = (float)errors / (float)N_INFO;

    printf("\n=== BER ===\n");
    printf("So bit thong tin = %d\n", N_INFO);
    printf("So bit loi       = %d\n", errors);
    printf("BER              = %.8f\n", ber);

    free(dma_buf);

    return 0;
}
