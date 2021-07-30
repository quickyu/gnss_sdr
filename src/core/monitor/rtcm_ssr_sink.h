#ifndef GNSS_SDR_RTCM_SSR_SINK_H
#define GNSS_SDR_RTCM_SSR_SINK_H

#include <gnuradio/block.h>
#include <gnuradio/runtime_types.h> 
#include <fstream>

#include "gnss_block_interface.h"
#include "beidou_cnav3_navigation_message.h"

class rtcm_ssr_sink;

using rtcm_ssr_sink_sptr = gnss_shared_ptr<rtcm_ssr_sink>;

rtcm_ssr_sink_sptr make_rtcm_ssr_sink(int n_channels, int udp_port, std::string &udp_address, bool log_to_file);

class rtcm_ssr_sink : public gr::block {
public:
   ~rtcm_ssr_sink();
   void forecast(int noutput_items, gr_vector_int &ninput_items_required);
   int general_work(int noutput_items, gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
   friend rtcm_ssr_sink_sptr make_rtcm_ssr_sink(int n_channels, int udp_port, std::string &udp_address, bool log_to_file);

   rtcm_ssr_sink(int n_channels, int udp_port, std::string &udp_address, bool log_to_file);

   std::string timestamp();
   void write_log_file(std::ofstream &log_file, const cnav3_message_content &content);
   //std::string parser_message(int type, const uint8_t *data);

   int d_udp_port;
   int d_nchannels;
   bool d_log_to_file;

   std::string d_udp_address;
   std::vector<std::ofstream> d_log_files;
};

#endif