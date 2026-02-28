/*
 * dcs_decode.cc
 *   DCS (Digital Coded Squelch) decoder implementation.
 *
 * DCS is a continuous 134.4 bps NRZ subaudible bitstream (< 300 Hz).
 * Each 23-bit frame is a systematic (23,12) Golay codeword:
 *   bits [22..11] = 12 data bits (bits 8..0 = DCS code, bits 11..9 = 0)
 *   bits [10..0]  = 11 Golay check bits
 *
 * Generator polynomial: g(x) = x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1
 * (per EIA/TIA-603 standard for DCS subaudible coding)
 *
 * Algorithm:
 *   1. First-order IIR low-pass filter at ~300 Hz isolates the DCS tone
 *   2. Integration over each bit period + threshold for bit decision
 *   3. Zero-crossing clock recovery nudges the bit clock for better sync
 *   4. Dual sliding 23-bit windows (both bit orderings) feed Golay decode
 *   5. Two consecutive matching valid codewords required before callback
 */

#include "dcs_decode.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Standard DCS codes (decimal values converted from EIA-603 octal table).
 * 105 codes total.
 * -------------------------------------------------------------------------- */
static const uint16_t DCS_VALID_CODES[] = {
     19,  21,  22,  25,  26,  30,  35,  39,  41,  43,  44,  53,
     57,  58,  59,  60,
     76,  77,  78,  82,  85,  89,  90,  92,  99, 101, 106, 109,
    110, 114, 117, 122, 124,
    133, 138, 140, 147, 149, 150, 163, 164, 165, 166, 169, 170,
    173, 177, 179, 181, 182, 185, 188,
    198, 201, 205, 213, 217, 218, 227, 230, 233, 238, 244, 245, 249,
    265, 266, 267, 275, 281, 282, 293, 294, 298, 300, 301, 306, 308,
    309, 310,
    323, 326, 334, 339, 342, 346, 358, 373,
    390, 394, 404, 407, 409, 410, 428, 434, 436,
    451, 458, 467, 473, 474, 476, 483, 492
};
#define DCS_NUM_CODES  ((int)(sizeof(DCS_VALID_CODES) / sizeof(DCS_VALID_CODES[0])))

/* --------------------------------------------------------------------------
 * Golay (23,12) implementation
 * g(x) = x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1 = 0xC75
 * -------------------------------------------------------------------------- */
#define GOLAY_POLY  0xC75U   /* 12-bit representation including x^11 term */
#define GOLAY_SYN_INVALID 0xFFFFFFFFU

static uint32_t golay_syndrome(uint32_t word) {
    uint32_t reg = word & 0x7FFFFFU;  /* 23 bits */
    int i;
    for (i = 22; i >= 11; i--) {
        if ((reg >> i) & 1U) {
            reg ^= (GOLAY_POLY << (i - 11));
        }
    }
    return reg & 0x7FFU;  /* 11-bit syndrome */
}

/*
 * Build a 2048-entry syndrome → error-pattern table.
 * The (23,12,7) Golay code corrects up to 3 errors; the 2048 syndromes
 * map exactly to the 1 + C(23,1) + C(23,2) + C(23,3) = 2048 correctable
 * error patterns.
 */
static void build_syndrome_table(uint32_t *tbl) {
    int i, j, k;
    for (i = 0; i < 2048; i++) tbl[i] = GOLAY_SYN_INVALID;

    /* 0 errors */
    tbl[0] = 0;

    /* 1-bit errors */
    for (i = 0; i < 23; i++) {
        uint32_t e = 1U << i;
        uint32_t s = golay_syndrome(e);
        if (tbl[s] == GOLAY_SYN_INVALID) tbl[s] = e;
    }

    /* 2-bit errors */
    for (i = 0; i < 23; i++) {
        for (j = i + 1; j < 23; j++) {
            uint32_t e = (1U << i) | (1U << j);
            uint32_t s = golay_syndrome(e);
            if (tbl[s] == GOLAY_SYN_INVALID) tbl[s] = e;
        }
    }

    /* 3-bit errors */
    for (i = 0; i < 23; i++) {
        for (j = i + 1; j < 23; j++) {
            for (k = j + 1; k < 23; k++) {
                uint32_t e = (1U << i) | (1U << j) | (1U << k);
                uint32_t s = golay_syndrome(e);
                if (tbl[s] == GOLAY_SYN_INVALID) tbl[s] = e;
            }
        }
    }
}

static int is_valid_dcs_code(int code) {
    int i;
    for (i = 0; i < DCS_NUM_CODES; i++) {
        if (DCS_VALID_CODES[i] == (uint16_t)code) return 1;
    }
    return 0;
}

/*
 * Try to Golay-decode a 23-bit window.
 * The systematic layout assumed is: bits [22..11] = data, bits [10..0] = parity.
 * Returns 1 if a valid recognized DCS code is found, fills *out_code and *out_inverted.
 */
static int try_decode_word(const uint32_t *syn_tbl, uint32_t word,
                           int *out_code, int *out_inverted, int polarity_inv) {
    uint32_t s = golay_syndrome(word);
    if (s < 2048U && syn_tbl[s] != GOLAY_SYN_INVALID) {
        uint32_t corrected = word ^ syn_tbl[s];
        int data = (int)((corrected >> 11) & 0xFFFU);
        /* Bits 11..9 of data must be 0 for any standard DCS code */
        if ((data & 0xE00) == 0 && is_valid_dcs_code(data)) {
            *out_code = data;
            *out_inverted = polarity_inv;
            return 1;
        }
    }
    return 0;
}

/* --------------------------------------------------------------------------
 * Decoder state
 * -------------------------------------------------------------------------- */
struct dcs_decoder {
    /* Low-pass filter (first-order IIR, cutoff ~300 Hz) */
    float lp_alpha;
    float lp_state;
    float lp_prev;      /* previous filtered sample, for zero-crossing detection */

    /* Bit clock */
    float samples_per_bit;
    float bit_phase;    /* fractional sample count within current bit period */
    float bit_accum;    /* accumulated filtered samples for current bit */

    /* Sliding windows.
     * window_a: newest bit enters at MSB  (>>1 | bit<<22)
     * window_b: newest bit enters at LSB  (<<1 | bit, masked to 23 bits)
     * Trying both accommodates uncertainty in DCS bit-transmission order.
     */
    uint32_t window_a;
    uint32_t window_b;

    /* Confirmation: require 2 consecutive matching valid codewords */
    int last_code;
    int last_inverted;
    int confirm_count;

    /* Golay syndrome table */
    uint32_t syndrome_tbl[2048];

    /* Callback */
    dcs_callback_t callback;
    void *callback_ctx;
};

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

dcs_decoder_t *dcs_decoder_new(int sampleRate) {
    dcs_decoder_t *dec = (dcs_decoder_t *)calloc(1, sizeof(dcs_decoder_t));
    if (!dec) return NULL;

    /* First-order IIR LP: alpha = 1 - e^(-2*pi*fc/fs), fc = 300 Hz */
    dec->lp_alpha  = 1.0f - expf(-2.0f * 3.14159265f * 300.0f / (float)sampleRate);
    dec->lp_state  = 0.0f;
    dec->lp_prev   = 0.0f;

    dec->samples_per_bit = (float)sampleRate / 134.4f;
    dec->bit_phase = 0.0f;
    dec->bit_accum = 0.0f;

    dec->window_a = 0;
    dec->window_b = 0;

    dec->last_code    = -1;
    dec->last_inverted = 0;
    dec->confirm_count = 0;

    build_syndrome_table(dec->syndrome_tbl);

    dec->callback     = NULL;
    dec->callback_ctx = NULL;

    return dec;
}

void dcs_decoder_delete(dcs_decoder_t *dec) {
    if (dec) free(dec);
}

void dcs_decoder_set_callback(dcs_decoder_t *dec, dcs_callback_t cb, void *ctx) {
    if (!dec) return;
    dec->callback     = cb;
    dec->callback_ctx = ctx;
}

void dcs_decoder_process_samples(dcs_decoder_t *dec,
                                 const dcs_sample_t *samples,
                                 int numSamples) {
    int i;
    float alpha        = dec->lp_alpha;
    float one_minus_a  = 1.0f - alpha;

    for (i = 0; i < numSamples; i++) {
        /* --- Low-pass filter --- */
        float filtered = alpha * samples[i] + one_minus_a * dec->lp_state;
        dec->lp_state = filtered;

        /* --- Zero-crossing clock recovery ---
         * When a zero crossing occurs, nudge the bit clock so the
         * sampling point lands near the centre of each bit period.
         */
        if ((dec->lp_prev < 0.0f) != (filtered < 0.0f)) {
            float half = dec->samples_per_bit * 0.5f;
            if (dec->bit_phase < half) {
                /* Crossing in first half: clock edge is early, delay slightly */
                dec->bit_phase += dec->samples_per_bit * 0.05f;
            } else {
                /* Crossing in second half: clock edge is late, advance slightly */
                dec->bit_phase -= dec->samples_per_bit * 0.05f;
            }
        }
        dec->lp_prev = filtered;

        /* --- Integrate sample into current bit accumulator --- */
        dec->bit_accum += filtered;
        dec->bit_phase  += 1.0f;

        /* --- Bit boundary reached? --- */
        if (dec->bit_phase >= dec->samples_per_bit) {
            dec->bit_phase -= dec->samples_per_bit;

            /* Threshold: positive average → 1, negative → 0 */
            int bit = (dec->bit_accum > 0.0f) ? 1 : 0;
            dec->bit_accum = 0.0f;

            /* Update both sliding windows */
            dec->window_a = (dec->window_a >> 1) | ((uint32_t)bit << 22);
            dec->window_b = ((dec->window_b << 1) | (uint32_t)bit) & 0x7FFFFFU;

            /* Try to decode both windows, both polarities */
            int code = 0, inverted = 0, found = 0;

            if (!found) found = try_decode_word(dec->syndrome_tbl,
                                                dec->window_a, &code, &inverted, 0);
            if (!found) found = try_decode_word(dec->syndrome_tbl,
                                                (~dec->window_a) & 0x7FFFFFU,
                                                &code, &inverted, 1);
            if (!found) found = try_decode_word(dec->syndrome_tbl,
                                                dec->window_b, &code, &inverted, 0);
            if (!found) found = try_decode_word(dec->syndrome_tbl,
                                                (~dec->window_b) & 0x7FFFFFU,
                                                &code, &inverted, 1);

            if (found) {
                if (code == dec->last_code && inverted == dec->last_inverted) {
                    dec->confirm_count++;
                    /* Fire callback on first confirmation (2nd consecutive match) */
                    if (dec->confirm_count == 2 && dec->callback) {
                        dec->callback(code, inverted, dec->callback_ctx);
                    }
                    /* Continue firing every bit for ongoing squelch refresh */
                    if (dec->confirm_count > 2 && dec->callback) {
                        dec->callback(code, inverted, dec->callback_ctx);
                    }
                } else {
                    dec->last_code    = code;
                    dec->last_inverted = inverted;
                    dec->confirm_count = 1;
                }
            } else {
                /* Decay confirmation counter when no valid code seen */
                if (dec->confirm_count > 0) dec->confirm_count--;
            }
        }
    }
}
