#ifndef GNSS_SDR_BEIDOU_B2B_TELEMETRY_DECODER_H
#define GNSS_SDR_BEIDOU_B2B_TELEMETRY_DECODER_H

#include "beidou_b2b_telemetry_decoder_gs.h"
#include "gnss_satellite.h"  // for Gnss_Satellite
#include "gnss_synchro.h"
#include "telemetry_decoder_interface.h"
#include "tlm_conf.h"
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <cstddef>                   // for size_t
#include <string>


/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for BEIDOU B2b
 */
class BeidouB2bTelemetryDecoder : public TelemetryDecoderInterface
{
public:
    BeidouB2bTelemetryDecoder(
        const ConfigurationInterface *configuration,
        const std::string &role, unsigned int in_streams,
        unsigned int out_streams);

    ~BeidouB2bTelemetryDecoder() = default;

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    void set_satellite(const Gnss_Satellite &satellite) override;

    inline std::string role() override { return role_; }

    inline std::string implementation() override
    {
        return "BEIDOU_B2b_Telemetry_Decoder";
    }

    inline void set_channel(int channel) override
    {
        telemetry_decoder_->set_channel(channel);
    }

    inline void reset() override
    {
        telemetry_decoder_->reset();
    }

    inline size_t item_size() override { return sizeof(Gnss_Synchro); }

private:
    beidou_b2b_telemetry_decoder_gs_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    Tlm_Conf tlm_parameters_;
    std::string role_;
    int channel_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

/** \} */
/** \} */
#endif
