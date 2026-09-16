#ifndef RECEIVER_CHAIN_H
#define RECEIVER_CHAIN_H

#include <stdint.h>

// Chay toan bo chuoi xu ly: frame sync -> doppler -> CFO -> RLS -> QPSK demod
// -> deinterleave -> Viterbi decode.
//
// dma_buf: con tro toi du lieu I/Q da capture (dinh dang: I 32-bit, Q 32-bit
//          lien tiep cho moi mau, tong cong n_samples*2 phan tu int32_t).
//          Con tro nay CO THE tro thang vao DDR4 that (tren Zynq) hoac vao
//          buffer da doc tu file (khi test tren PC) - ham nay khong quan tam
//          nguon goc, chi doc du lieu.
// n_samples: so mau I/Q (KHONG PHAI so phan tu int32_t - moi mau = 2 phan tu)
// decoded_info_out: buffer output, PHAI duoc cap phat truoc voi kich thuoc >= N_INFO
//                   (N_INFO dinh nghia trong config.h)
//
// Tra ve: 0 neu thanh cong, -1 neu loi (vd khong tim thay frame, sync sai...)
int run_receiver_chain(const int32_t *dma_buf, int n_samples,
                         unsigned char *decoded_info_out);

#endif // RECEIVER_CHAIN_H
