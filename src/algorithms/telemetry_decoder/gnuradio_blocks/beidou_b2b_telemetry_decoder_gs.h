#ifndef GNSS_SDR_BEIDOU_B2B_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_BEIDOU_B2B_TELEMETRY_DECODER_GS_H

#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "tlm_conf.h"

#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>  // for block
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <array>
#include <cstdint>
#include <fstream>
#include <string>


/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */


class beidou_b2b_telemetry_decoder_gs;

using beidou_b2b_telemetry_decoder_gs_sptr =
    gnss_shared_ptr<beidou_b2b_telemetry_decoder_gs>;

beidou_b2b_telemetry_decoder_gs_sptr beidou_b2b_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf);

/*!
 * \brief This class implements a block that decodes the BeiDou DNAV data.
 */
class beidou_b2b_telemetry_decoder_gs : public gr::block
{
public:
    ~beidou_b2b_telemetry_decoder_gs();                   //!< Class destructor
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int channel);                        //!< Set receiver's channel
    void reset();

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend beidou_b2b_telemetry_decoder_gs_sptr beidou_b2b_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        const Tlm_Conf &conf);

    beidou_b2b_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf);

    void save_frame_symbol(std::string &symbol);
    bool crc_test(std::string const &frame) const;
 
    Gnss_Satellite d_satellite;

    std::vector<int32_t> d_preamble_samples;
    boost::circular_buffer<float> d_symbol_history;

    std::ofstream d_dump_file;
    std::string d_dump_filename;
  
    int32_t d_channel;
    int32_t d_preamble_period_symbols;
    uint32_t d_stat;
    uint32_t d_required_symbols;
    uint32_t d_samples_per_preamble;
    uint32_t d_preamble_index;
    uint64_t d_sample_counter;
    uint64_t d_last_valid_preamble;
    uint32_t d_max_symbols_without_valid_frame;
    uint32_t d_crc_error_counter;
    bool d_sent_tlm_failed_msg;
    bool d_flag_preamble;
    bool d_flag_PLL_180_deg_phase_locked;
    bool d_dump;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B2B_TELEMETRY_DECODER_GS_H
