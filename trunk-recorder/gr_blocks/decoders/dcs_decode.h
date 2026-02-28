/*
 * dcs_decode.h
 *   Header for DCS (Digital Coded Squelch) decoder.
 *
 * DCS transmits a continuous 134.4 bps NRZ bitstream below 300 Hz.
 * Each 23-bit frame is a (23,12) Golay codeword: 12 data bits (9-bit
 * DCS code number + 3 zero bits) followed by 11 Golay check bits.
 *
 * Codes are reported in decimal (e.g. octal 023 = decimal 19).
 * The inverted flag is set when the signal polarity is reversed
 * (corresponding to the "N" suffix in D023N notation).
 */

#ifndef DCS_DECODE_H
#define DCS_DECODE_H

#include "dcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dcs_decoder dcs_decoder_t;

/*
 * Callback fired on each confirmed DCS decode.
 *   code     - decimal DCS code number (e.g. 19 for D023, 21 for D025)
 *   inverted - 1 if inverted polarity ("N" suffix), 0 for normal
 *   context  - user context pointer passed to dcs_decoder_set_callback
 */
typedef void (*dcs_callback_t)(int code, int inverted, void *context);

/*
 * dcs_decoder_new
 *   Allocate and initialize a new DCS decoder.
 *   sampleRate - audio sample rate in Hz (typically 16000 or 96000)
 *   Returns pointer to decoder, or NULL on allocation failure.
 */
dcs_decoder_t *dcs_decoder_new(int sampleRate);

/*
 * dcs_decoder_delete
 *   Free a decoder created with dcs_decoder_new.
 */
void dcs_decoder_delete(dcs_decoder_t *decoder);

/*
 * dcs_decoder_process_samples
 *   Process incoming float audio samples.
 *   The callback (if set) will be called from within this function
 *   whenever a valid DCS code is confirmed.
 */
void dcs_decoder_process_samples(dcs_decoder_t *decoder,
                                 const dcs_sample_t *samples,
                                 int numSamples);

/*
 * dcs_decoder_set_callback
 *   Set the callback invoked on each successful decode.
 */
void dcs_decoder_set_callback(dcs_decoder_t *decoder,
                              dcs_callback_t callback,
                              void *context);

#ifdef __cplusplus
}
#endif

#endif /* DCS_DECODE_H */
