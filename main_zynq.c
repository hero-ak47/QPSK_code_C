// ============================================================================
// main_zynq.c - Entry point THAT tren Zynq (bare-metal, Vitis/SDK).
//
// Luong hoat dong:
//   [Vong lap] -> cho nut bam -> arm DMA + phat start_pulse -> cho DMA xong
//               -> chay toan bo receiver chain (frame sync/doppler/CFO/RLS/
//                  Viterbi) truc tiep tren du lieu vua capture trong DDR4
//               -> in ket qua -> quay lai cho nut bam tiep theo
//
// *** CHI BIEN DICH TREN VITIS/SDK VOI BSP CUA ZYNQ, KHONG CHAY TREN PC ***
// ============================================================================

#include <stdio.h>
#include <stdint.h>
#include "xaxidma.h"
#include "xgpio.h"
#include "xparameters.h"
#include "xil_printf.h"

#include "capture_trigger.h"
#include "receiver_chain.h"
#include "config.h"

// *** SUA CAC ID NAY CHO KHOP VOI xparameters.h THUC TE CUA BAN ***
#define DMA_DEV_ID       XPAR_AXIDMA_0_DEVICE_ID
#define GPIO_DEV_ID      XPAR_AXI_GPIO_0_DEVICE_ID
#define BUTTON_CHANNEL   1
#define START_CHANNEL    2

#define DDR_BUF_ADDR     0x400000000ULL
#define CAPTURE_SEC      5
#define N_SAMPLES        (FS * CAPTURE_SEC)
#define BYTES_PER_SAMPLE 8
#define BUF_LEN          (N_SAMPLES * BYTES_PER_SAMPLE)

int main(void) {
    XAxiDma AxiDma;
    XGpio   Gpio;
    int     status;

    status = capture_trigger_init(&AxiDma, &Gpio, DMA_DEV_ID, GPIO_DEV_ID);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: capture_trigger_init that bai, status=%d\r\n", status);
        return -1;
    }

    xil_printf("He thong san sang. Cho nut bam de bat dau capture...\r\n");

    u32 button_prev = 0;
    unsigned char decoded_info[N_INFO];

    while (1) {
        if (capture_trigger_check_button_edge(&Gpio, BUTTON_CHANNEL, &button_prev)) {

            xil_printf("Nut bam duoc nhan! Bat dau capture...\r\n");

            status = capture_trigger_run(&AxiDma, &Gpio,
                                           (UINTPTR)DDR_BUF_ADDR, BUF_LEN,
                                           START_CHANNEL);
            if (status != XST_SUCCESS) {
                xil_printf("ERROR: capture_trigger_run that bai, status=%d\r\n", status);
                continue;   // quay lai cho nut bam tiep theo, khong crash he thong
            }

            xil_printf("Capture hoan tat (%lu byte). Dang chay receiver chain...\r\n",
                        (unsigned long)BUF_LEN);

            // Chay toan bo DSP chain TRUC TIEP tren du lieu trong DDR4
            // (khong can copy ra buffer rieng - con tro tro thang vao DDR4)
            const int32_t *dma_buf_ptr = (const int32_t*)(uintptr_t)DDR_BUF_ADDR;
            int rx_status = run_receiver_chain(dma_buf_ptr, N_SAMPLES, decoded_info);

            if (rx_status == 0) {
                xil_printf("=== GIAI MA THANH CONG ===\r\n");
            } else {
                xil_printf("=== GIAI MA THAT BAI (status=%d) - kiem tra frame sync/CFO ===\r\n",
                            rx_status);
            }

            xil_printf("San sang cho lan bam nut tiep theo...\r\n");
        }
    }

    return 0;
}
