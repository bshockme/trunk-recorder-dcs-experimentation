/* -*- c++ -*- */
/*
 * dcs_squelch_ff_impl.h
 *   Implementation header for dcs_squelch_ff.
 */

#ifndef INCLUDED_DCS_SQUELCH_FF_IMPL_H
#define INCLUDED_DCS_SQUELCH_FF_IMPL_H

#include "dcs_squelch_ff.h"
#include "decoders/dcs_decode.h"
#include <boost/log/trivial.hpp>

namespace gr {
namespace blocks {

class dcs_squelch_ff_impl : public dcs_squelch_ff {
private:
    dcs_decoder_t *d_dcs_decoder;

    int   d_target_code;
    bool  d_target_inverted;

    bool  d_squelch_open;
    int   d_tail_samples;       /* remaining tail samples */
    int   d_tail_samples_max;   /* tail length in samples */

    /* Called from within dcs_decoder_process_samples when code matches */
    static void dcs_callback(int code, int inverted, void *context);

public:
#if GNURADIO_VERSION < 0x030900
    typedef boost::shared_ptr<dcs_squelch_ff_impl> sptr;
#else
    typedef std::shared_ptr<dcs_squelch_ff_impl> sptr;
#endif

    static sptr make(int sample_rate, int target_code,
                     bool target_inverted, float tail_ms);

    dcs_squelch_ff_impl(int sample_rate, int target_code,
                        bool target_inverted, float tail_ms);
    ~dcs_squelch_ff_impl();

    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);

    void set_target_code(int code, bool inverted);
    bool is_open() const;
};

} /* namespace blocks */
} /* namespace gr */

#endif /* INCLUDED_DCS_SQUELCH_FF_IMPL_H */
