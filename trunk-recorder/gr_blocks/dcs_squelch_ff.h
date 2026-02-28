/* -*- c++ -*- */
/*
 * dcs_squelch_ff.h
 *   GNU Radio sync_block that gates float audio based on detected DCS code.
 *
 * Takes the FM-demodulated audio stream as input and produces the same audio
 * on output, but zeroes the output whenever the configured DCS code is NOT
 * present.  Mirrors the behaviour of gr::analog::ctcss_squelch_ff for
 * CTCSS, but operates on the digital DCS (134.4 bps Golay-coded) signal.
 *
 * Placement in the flowgraph (same position as ctcss_squelch_ff):
 *   de-emphasis → [dcs_squelch_ff] → decim_audio → ...
 *
 * Parameters:
 *   sample_rate   - input sample rate in Hz (typically system_channel_rate)
 *   target_code   - DCS code in decimal (e.g. 19 for D023, 21 for D025)
 *   target_inverted - true for inverted polarity ("N" suffix, e.g. D023N)
 *   tail_ms       - squelch tail in milliseconds after last detected code
 *                   (default 250 ms)
 */

#ifndef INCLUDED_DCS_SQUELCH_FF_H
#define INCLUDED_DCS_SQUELCH_FF_H

#include <gnuradio/blocks/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace blocks {

class BLOCKS_API dcs_squelch_ff : virtual public sync_block {
public:
#if GNURADIO_VERSION < 0x030900
    typedef boost::shared_ptr<dcs_squelch_ff> sptr;
#else
    typedef std::shared_ptr<dcs_squelch_ff> sptr;
#endif

    static sptr make(int sample_rate,
                     int target_code,
                     bool target_inverted,
                     float tail_ms = 250.0f);

    virtual void set_target_code(int code, bool inverted) = 0;
    virtual bool is_open() const = 0;
};

} /* namespace blocks */
} /* namespace gr */

#endif /* INCLUDED_DCS_SQUELCH_FF_H */
