/* -*- c++ -*- */
/*
 * dcs_squelch_ff_impl.cc
 *   GNU Radio float→float squelch gate driven by DCS code detection.
 *
 * The DCS decoder runs on every input sample.  Whenever the target code
 * is confirmed the squelch opens and a tail timer is (re)started.  Audio
 * passes through unchanged while the squelch is open; zeros are output
 * while it is closed.
 */

#include "dcs_squelch_ff_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>

namespace gr {
namespace blocks {

/* -------------------------------------------------------------------------
 * Factory / constructor / destructor
 * ---------------------------------------------------------------------- */

dcs_squelch_ff::sptr
dcs_squelch_ff_impl::make(int sample_rate, int target_code,
                          bool target_inverted, float tail_ms) {
    return gnuradio::get_initial_sptr(
        new dcs_squelch_ff_impl(sample_rate, target_code, target_inverted, tail_ms));
}

dcs_squelch_ff_impl::dcs_squelch_ff_impl(int sample_rate, int target_code,
                                         bool target_inverted, float tail_ms)
    : sync_block("dcs_squelch_ff",
                 io_signature::make(1, 1, sizeof(float)),
                 io_signature::make(1, 1, sizeof(float))),
      d_target_code(target_code),
      d_target_inverted(target_inverted),
      d_squelch_open(false),
      d_tail_samples(0) {

    d_tail_samples_max = (int)((float)sample_rate * tail_ms / 1000.0f);

    d_dcs_decoder = dcs_decoder_new(sample_rate);
    if (d_dcs_decoder) {
        dcs_decoder_set_callback(d_dcs_decoder, dcs_squelch_ff_impl::dcs_callback, this);
    }

    BOOST_LOG_TRIVIAL(info) << "DCS squelch: target D"
                            << std::oct << target_code << std::dec
                            << (target_inverted ? "N" : "")
                            << "  tail=" << tail_ms << " ms"
                            << "  sample_rate=" << sample_rate;
}

dcs_squelch_ff_impl::~dcs_squelch_ff_impl() {
    dcs_decoder_delete(d_dcs_decoder);
}

/* -------------------------------------------------------------------------
 * DCS decoder callback
 * Called from within dcs_decoder_process_samples() when a valid code is seen.
 * ---------------------------------------------------------------------- */
void dcs_squelch_ff_impl::dcs_callback(int code, int inverted, void *context) {
    dcs_squelch_ff_impl *self = static_cast<dcs_squelch_ff_impl *>(context);
    if (code == self->d_target_code && (inverted != 0) == self->d_target_inverted) {
        self->d_squelch_open = true;
        self->d_tail_samples = self->d_tail_samples_max;
    }
}

/* -------------------------------------------------------------------------
 * GNU Radio work() — processes one buffer of samples
 * ---------------------------------------------------------------------- */
int dcs_squelch_ff_impl::work(int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items) {
    const float *in  = static_cast<const float *>(input_items[0]);
    float       *out = static_cast<float *>(output_items[0]);

    /* Run DCS decoder over input — updates d_squelch_open and d_tail_samples
     * via the callback for any matching code detected in this block.         */
    if (d_dcs_decoder) {
        dcs_decoder_process_samples(d_dcs_decoder, in, noutput_items);
    }

    /* Gate output sample-by-sample, decrementing the tail timer */
    for (int i = 0; i < noutput_items; i++) {
        if (d_squelch_open) {
            out[i] = in[i];
            if (d_tail_samples > 0) {
                d_tail_samples--;
                if (d_tail_samples == 0) {
                    d_squelch_open = false;
                }
            }
        } else {
            out[i] = 0.0f;
        }
    }

    return noutput_items;
}

/* -------------------------------------------------------------------------
 * Public control methods
 * ---------------------------------------------------------------------- */
void dcs_squelch_ff_impl::set_target_code(int code, bool inverted) {
    d_target_code     = code;
    d_target_inverted = inverted;
    d_squelch_open    = false;
    d_tail_samples    = 0;
}

bool dcs_squelch_ff_impl::is_open() const {
    return d_squelch_open;
}

} /* namespace blocks */
} /* namespace gr */
