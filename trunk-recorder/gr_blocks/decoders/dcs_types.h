/*
 * dcs_types.h
 *   Type definitions for DCS (Digital Coded Squelch) decoder.
 *
 * DCS is a 134.4 bps NRZ subaudible (< 300 Hz) signaling system using
 * a (23,12) Golay code for error correction.
 */

#ifndef DCS_TYPES_H
#define DCS_TYPES_H

typedef float    dcs_sample_t;
typedef unsigned int   dcs_u32_t;
typedef unsigned short dcs_u16_t;
typedef unsigned char  dcs_u8_t;
typedef int            dcs_int_t;
typedef float          dcs_float_t;

#define DCS_SAMPLE_FORMAT_FLOAT

#endif /* DCS_TYPES_H */
