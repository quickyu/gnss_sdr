#include "beidou_b2b_dll_pll_tracking.h"
#include "Beidou_B2b.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <array>

using google::LogMessage;

BeidouB2bDllPllTracking::BeidouB2bDllPllTracking(
    const ConfigurationInterface* configuration, const std::string& role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Dll_Pll_Conf trk_params = Dll_Pll_Conf();
    DLOG(INFO) << "role " << role;
    trk_params.SetFromConfiguration(configuration, role);

    const auto vector_length = static_cast<int>(std::round(static_cast<double>(trk_params.fs_in) / (BEIDOU_B2B_CODE_RATE_CPS / BEIDOU_B2B_CODE_LENGTH_CHIPS)));
    trk_params.vector_length = vector_length;
    trk_params.track_pilot = configuration->property(role + ".track_pilot", false);
    if (trk_params.extend_correlation_symbols < 1)
        {
            trk_params.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: BEIDOU B2b. extend_correlation_symbols must be bigger than 1. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << '\n';
        }
    else if (trk_params.extend_correlation_symbols > 20)
        {
            trk_params.extend_correlation_symbols = 20;
            std::cout << TEXT_RED << "WARNING: BEIDOU B2b. extend_correlation_symbols must be lower than 21. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << '\n';
        }
    trk_params.system = 'C';
    const std::array<char, 3> sig_{'B', '2', '\0'};
    std::memcpy(trk_params.signal, sig_.data(), 3);

    // ################# Make a GNU Radio Tracking block object ################
    if (trk_params.item_type == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = dll_pll_veml_make_tracking(trk_params);
        }
    else
        {
            item_size_ = 0;
            LOG(WARNING) << trk_params.item_type << " unknown tracking item type.";
        }
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void BeidouB2bDllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}


void BeidouB2bDllPllTracking::stop_tracking()
{
    tracking_->stop_tracking();
}


/*
 * Set tracking channel unique ID
 */
void BeidouB2bDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}


void BeidouB2bDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}


void BeidouB2bDllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void BeidouB2bDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr BeidouB2bDllPllTracking::get_left_block()
{
    return tracking_;
}


gr::basic_block_sptr BeidouB2bDllPllTracking::get_right_block()
{
    return tracking_;
}
