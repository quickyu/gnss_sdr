#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <boost/format.hpp> 
#include <glog/logging.h>

#include "rtcm_ssr_sink.h"
#include "Beidou_CNAV3.h"

rtcm_ssr_sink_sptr make_rtcm_ssr_sink(int n_channels, int udp_port, std::string &udp_address, bool log_to_file)
{
   return rtcm_ssr_sink_sptr(new rtcm_ssr_sink(n_channels, udp_port, udp_address, log_to_file));
}

rtcm_ssr_sink::rtcm_ssr_sink(int n_channels, int udp_port, std::string &udp_address, bool log_to_file) : 
               gr::block("rtcm_ssr_sink",
               gr::io_signature::make(n_channels, n_channels, 512),
               gr::io_signature::make(0, 0, 0))
{
   d_nchannels = n_channels;
   d_udp_port = udp_port;
   d_udp_address = udp_address;

   d_log_to_file = log_to_file;

   if (log_to_file) {
      for (int i = 0; i < n_channels; i++) {
         d_log_files.push_back(std::ofstream());
      }

      for (int i = 0; i < n_channels; i++) {
         std::string filename = "beidou_ssr_ch" + std::to_string(i) + ".log";

         try {
            d_log_files[i].open(filename.c_str(), std::ios::out | std::ios::trunc);
         } catch (const std::ifstream::failure &e) {
            LOG(WARNING) << "channel " << i << " Exception opening ssr log file " << e.what();
         }   
      }
   }
}

rtcm_ssr_sink::~rtcm_ssr_sink()
{
   for (auto &of : d_log_files) {
      if (of.is_open()) {
         try {
            of.close();
         } catch (const std::exception &ex) {
            LOG(WARNING) << "Exception in destructor closing the log file " << ex.what();
         }
      }
   }
}

void rtcm_ssr_sink::forecast(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items_required)
{
   for (int32_t channel_index = 0; channel_index < d_nchannels; channel_index++) {
      // Set the required number of inputs to 0 so that a lone input on any channel can be pushed to UDP
      ninput_items_required[channel_index] = 0;
   }
}

std::string rtcm_ssr_sink::timestamp()
{
   using namespace std::chrono;
   using clock = system_clock;
    
   const auto current_time_point {clock::now()};
   const auto current_time {clock::to_time_t (current_time_point)};
   const auto current_localtime {*std::localtime (&current_time)};
   const auto current_time_since_epoch {current_time_point.time_since_epoch()};
   const auto current_milliseconds {duration_cast<milliseconds> (current_time_since_epoch).count() % 1000};
    
   std::ostringstream stream;
   stream << std::put_time (&current_localtime, "%T") << "." << std::setw (3) << std::setfill ('0') << current_milliseconds;
   return stream.str();
}

int rtcm_ssr_sink::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items, gr_vector_void_star &output_items __attribute__((unused)))
{
   const char **in = reinterpret_cast<const char **>(&input_items[0]);

   for (int channel_index = 0; channel_index < d_nchannels; channel_index++) {
      for (int item_index = 0; item_index < ninput_items[channel_index]; item_index++) {
         std::string frame_bits(in[channel_index] + item_index*512, BEIDOU_CNAV3_FRAME_BITS);

         uint32_t prn = std::bitset<6>(frame_bits.substr(0, 6)).to_ulong();
         uint32_t ppp_status = std::bitset<6>(frame_bits.substr(6, 6)).to_ulong();
         uint32_t msg_type = std::bitset<6>(frame_bits.substr(12, 6)).to_ulong();

         std::string msg_info = timestamp() + "  Beidou CNAV3 message  " + "  prn: " + std::to_string(prn) + 
            "  ppp status: " + std::to_string(ppp_status) + "  type: " + std::to_string(msg_type);

         std::cout << msg_info << std::endl;
         if (d_log_to_file)
            d_log_files[channel_index] << msg_info << std::endl;

         switch (msg_type) {
         case CNAV3_MSG_1: 
         {
            beidou_cnav3_navigation_message::message1_data msg1 = 
               cnav3_message.message1_decode(frame_bits.substr(12, BEIDOU_CNAV3_DATA_LENGTH));
            if (d_log_to_file) {
               d_log_files[channel_index] << msg1 << std::endl;
            }   
            break;
         }   

         case CNAV3_MSG_2: 
         {
            beidou_cnav3_navigation_message::message2_data msg2 = 
               cnav3_message.message2_decode(frame_bits.substr(12, BEIDOU_CNAV3_DATA_LENGTH));
            if (d_log_to_file) {
               d_log_files[channel_index] << msg2 << std::endl;
            }   
            break;
         }   

         case CNAV3_MSG_3:
         {
            beidou_cnav3_navigation_message::message3_data msg3 = 
               cnav3_message.message3_decode(frame_bits.substr(12, BEIDOU_CNAV3_DATA_LENGTH));
            if (d_log_to_file) {
               d_log_files[channel_index] << msg3 << std::endl;
            }   
            break;
         }   

         case CNAV3_MSG_4: 
         {
            beidou_cnav3_navigation_message::message4_data msg4 = 
               cnav3_message.message4_decode(frame_bits.substr(12, BEIDOU_CNAV3_DATA_LENGTH));
            if (d_log_to_file) {
               d_log_files[channel_index] << msg4 << std::endl;
            }   
            break;
         }   

         default: 
            if (d_log_to_file) {
               d_log_files[channel_index] << std::endl;
            } 
         }
      }

      consume(channel_index, ninput_items[channel_index]);
   }

   return 0;
}
