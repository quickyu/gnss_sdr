#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <cstddef>          // for size_t
#include <cstdlib>          // for abs
#include <exception>        // for exception
#include <iostream>         // for cout
#include <memory>           // for shared_ptr, make_shared

#include "beidou_cnav3_navigation_message.h"
#include "beidou_b2b_telemetry_decoder_gs.h"
#include "Beidou_B2b.h"
#include "Beidou_CNAV3.h"
#include "display.h"
#include "gnss_synchro.h"
#include "tlm_utils.h"
#include "timestamp.h"

#define CRC_ERROR_LIMIT 6

beidou_b2b_telemetry_decoder_gs_sptr
beidou_b2b_make_telemetry_decoder_gs(const Gnss_Satellite &satellite,
    const Tlm_Conf &conf __attribute__((unused)))
{
    return beidou_b2b_telemetry_decoder_gs_sptr(new beidou_b2b_telemetry_decoder_gs(satellite, conf));
}

beidou_b2b_telemetry_decoder_gs::beidou_b2b_telemetry_decoder_gs(
    const Gnss_Satellite &satellite, const Tlm_Conf &conf __attribute__((unused)))
    : gr::block("beidou_b2b_telemetry_decoder_gs",
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
          gr::io_signature::make(1, 1, sizeof(struct cnav3_message_content)))
{
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "Initializing BeiDou B2b Telemetry Decoding for satellite " << this->d_satellite;

    d_channel = 0;
    d_stat = 0;
    d_samples_per_preamble = BEIDOU_CNAV3_PREAMBLE_SYMBOLS;
    d_required_symbols = BEIDOU_CNAV3_TELEMETRY_MESSAGE_SYMBOLS;
    d_sample_counter = 0;
    d_preamble_index = 0;
    d_last_valid_preamble = 0;
    d_max_symbols_without_valid_frame = BEIDOU_CNAV3_TELEMETRY_MESSAGE_SYMBOLS * 60;
    d_preamble_period_symbols = BEIDOU_CNAV3_TELEMETRY_MESSAGE_SYMBOLS;
    d_crc_error_counter = 0;

    d_sent_tlm_failed_msg = false;
    d_flag_preamble = false;
    d_flag_PLL_180_deg_phase_locked = false;

    d_symbol_history.set_capacity(d_required_symbols + 1);

    d_preamble_samples.reserve(d_samples_per_preamble);
    for (int i = 0; i < d_samples_per_preamble; i++) {
        d_preamble_samples[i] = BEIDOU_CNAV3_PREAMBLE[i] == '1' ? -1 : 1;
    }

    d_dump_filename = conf.dump_filename;
    d_dump = conf.dump;
}

beidou_b2b_telemetry_decoder_gs::~beidou_b2b_telemetry_decoder_gs()
{
    DLOG(INFO) << "BeiDou B2b Telemetry decoder block (channel " << d_channel << ") destructor called.";

    if (d_dump_file.is_open() == true) {
        try {
            d_dump_file.close();
        } catch (const std::exception &ex) {
            LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
        }
    }    
}

void beidou_b2b_telemetry_decoder_gs::set_satellite(
    const Gnss_Satellite &satellite)
{
    gr::thread::scoped_lock lock(d_setlock);

    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    d_last_valid_preamble = d_sample_counter;
    d_sent_tlm_failed_msg = false;

    DLOG(INFO) << "Setting decoder Finite State Machine to satellite "
               << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}

void beidou_b2b_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    LOG(INFO) << "Navigation channel set to " << channel;

    if (d_dump) {
        if (d_dump_file.is_open() == false) {
            try {
                d_dump_filename.append(std::to_string(d_channel));
                d_dump_filename.append(".log");
                d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::trunc);
                LOG(INFO) << "Beidou B2b Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename;
            } catch (const std::ifstream::failure &e) {
                LOG(WARNING) << "channel " << d_channel << " Exception opening cnav3 message dump file " << e.what();
            }
        }
    }
}

void beidou_b2b_telemetry_decoder_gs::reset()
{
    gr::thread::scoped_lock lock(d_setlock);

    d_last_valid_preamble = d_sample_counter;
    d_sent_tlm_failed_msg = false;
    d_stat = 0;

    DLOG(INFO) << "Beidou B2b Telemetry decoder reset for satellite " << d_satellite;
}

void beidou_b2b_telemetry_decoder_gs::save_frame_symbol(std::string &symbol)
{
    d_dump_file << timestamp() << std::endl;
    d_dump_file << symbol << std::endl << std::endl;
}

int beidou_b2b_telemetry_decoder_gs::general_work(
    int noutput_items __attribute__((unused)),
    gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    auto **out = reinterpret_cast<struct cnav3_message_content **>(&output_items[0]);           
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  

    Gnss_Synchro current_symbol{};  
    current_symbol = in[0][0];

    if (!current_symbol.Flag_valid_symbol_output) {
        consume_each(1);
        return 0;
    }

    d_symbol_history.push_back(current_symbol.Prompt_I);

    d_sample_counter++;  
    consume_each(1);
    d_flag_preamble = false;   

    if (d_sent_tlm_failed_msg == false) {
        if ((d_sample_counter - d_last_valid_preamble) > d_max_symbols_without_valid_frame) {
            const int message = 1;  // bad telemetry
            DLOG(INFO) << "sent telemetry fault. sat " << this->d_satellite;
            this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(message));
            d_sent_tlm_failed_msg = true;
        }
    }
   
    switch (d_stat) {
    case 0:
        {
            int32_t corr_value = 0;

            if (d_symbol_history.size() > d_required_symbols) {
                for (int32_t i = 0; i < d_samples_per_preamble; i++) {
                    if (d_symbol_history[i] < 0.0)  
                        corr_value -= d_preamble_samples[i];
                    else
                        corr_value += d_preamble_samples[i];
                }

                if (abs(corr_value) >= d_samples_per_preamble) {
                    d_preamble_index = d_sample_counter;  
                    std::cout << "Preamble detection for Beidou satellite " << d_satellite 
                        << " in channel " << d_channel << std::endl;
                    d_stat = 1;  
                }
            }

            break;
        }

    case 1:
        {
            int32_t corr_value = 0;

            if (d_symbol_history.size() > d_required_symbols) {
                for (int32_t i = 0; i < d_samples_per_preamble; i++) {
                    if (d_symbol_history[i] < 0.0) 
                        corr_value -= d_preamble_samples[i];
                    else
                        corr_value += d_preamble_samples[i];
                }    

                if (abs(corr_value) >= d_samples_per_preamble) {
                    const auto preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                    if (preamble_diff == d_preamble_period_symbols) {
                        d_preamble_index = d_sample_counter;  
                        d_crc_error_counter = 0;
                        d_flag_PLL_180_deg_phase_locked = corr_value < 0 ? true : false;
                        d_stat = 2;
                    } else if (preamble_diff > d_preamble_period_symbols)
                        d_stat = 0;  
                }
            }

            break;
        }

    case 2:    
        {
            if (d_sample_counter == 
                    d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols)) {   
                std::string data_bits_str = "";
                for (int i = 0; i < 498; i++) {
                    float symbol = d_symbol_history[i+d_samples_per_preamble];
                    if (d_flag_PLL_180_deg_phase_locked)
                        symbol = -symbol;

                    data_bits_str += symbol < 0.0 ? '1' : '0';
                }

                std::cout << "New Beidou CNAV3 message received in channel " << d_channel 
                        << " satellite " << d_satellite << std::endl ;

                if (d_dump) {
                    save_frame_symbol(data_bits_str);
                }        

                d_cnav3_message.frame_decode(data_bits_str);

                d_preamble_index = d_sample_counter;  

                if (d_cnav3_message.get_flag_crc_test() == true) {
                    d_crc_error_counter = 0;
                    gr::thread::scoped_lock lock(d_setlock);
                    d_last_valid_preamble = d_sample_counter;
                } else {
                    d_crc_error_counter++;
                    if (d_crc_error_counter > CRC_ERROR_LIMIT) {
                        std::cout << "Lost of frame sync SAT " << this->d_satellite;
                        d_stat = 0;
                    }
                } 
            }  

            break;
        }
    }

    if (d_cnav3_message.get_update_flag()) {
        *out[0] = d_cnav3_message.get_message();
        return 1;
    }

    return 0;
}
