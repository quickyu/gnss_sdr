#include "beidou_b2b_telemetry_decoder.h"
#include "configuration_interface.h"
#include <glog/logging.h>

BeidouB2bTelemetryDecoder::BeidouB2bTelemetryDecoder(
    const ConfigurationInterface *configuration, const std::string &role,
    unsigned int in_streams, unsigned int out_streams)
    : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    DLOG(INFO) << "role " << role;
    tlm_parameters_.SetFromConfiguration(configuration, role);
    // make telemetry decoder object
    telemetry_decoder_ = beidou_b2b_make_telemetry_decoder_gs(satellite_, tlm_parameters_);
    DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";
    channel_ = 0;
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void BeidouB2bTelemetryDecoder::set_satellite(const Gnss_Satellite &satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "TELEMETRY DECODER: satellite set to " << satellite_;
}


void BeidouB2bTelemetryDecoder::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void BeidouB2bTelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr BeidouB2bTelemetryDecoder::get_left_block()
{
    return telemetry_decoder_;
}


gr::basic_block_sptr BeidouB2bTelemetryDecoder::get_right_block()
{
    return telemetry_decoder_;
}
