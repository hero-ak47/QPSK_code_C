#ifndef VITERBI_DECODER_H
#define VITERBI_DECODER_H

void convolutional_encode(const unsigned char *info_bits, int n_bits,
                            int gen1, int gen2, int constraint_len,
                            unsigned char *out_bits);

void viterbi_build_trellis(int gen1, int gen2, int constraint_len);
void viterbi_free_trellis(void);
void viterbi_decode_hard(const unsigned char *received_bits, int n_time_steps,
                           unsigned char *decoded_bits_out);

#endif // VITERBI_DECODER_H
