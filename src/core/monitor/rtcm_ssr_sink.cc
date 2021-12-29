#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <boost/format.hpp> 
#include <glog/logging.h>

#include "rtcm_ssr_sink.h"
#include "Beidou_CNAV3.h"

#define ROUND(x)    ((int)floor((x)+0.5))

void rtcm_ssr_encoder::setbitu(int pos, int len, uint32_t data)
{
   if (len <= 0|| len > 32) 
      return;

   uint32_t mask = 1u<<(len-1);

   for (int i = pos; i < pos+len; i++, mask >>= 1) {
      if (data&mask) 
         _rtcm_buf[i/8] |= 1u<<(7-i%8); 
      else 
         _rtcm_buf[i/8] &= ~(1u<<(7-i%8));
   }
}

void rtcm_ssr_encoder::setbits(int pos, int len, int32_t data)
{
   if (data < 0) 
      data |= 1<<(len-1); 
   else 
      data &= ~(1<<(len-1)); 

   setbitu(pos, len, static_cast<uint32_t>(data));
}

uint32_t rtcm_ssr_encoder::crc24q(int length)
{
   uint32_t crc = 0;

   for (int i = 0; i < length; i++) 
      crc = ((crc<<8)&0xFFFFFF) ^ tbl_crc24q[(crc>>16)^_rtcm_buf[i]];

   return crc;
}

int rtcm_ssr_encoder::encode_ssr_head(int type, int sys, int tow, int nsat, int iod_ssr, int refd, int provid, int solid)
{
   int i = 24, msgno;
    
   switch (sys) {
   case SYS_GPS: 
      msgno = 1056 + type; 
      break;
   case SYS_BEIDOU: 
      msgno = 1257 + type; 
      break; 
   default: 
      return 0;
   }

   setbitu(i, 12, msgno); //message number
   i += 12;
   setbitu(i, 20, tow);  //epoch time (tow)
   i += 20; 
   setbitu(i, 4, 0); //update interval 
   i += 4; 
   setbitu(i, 1, 0); //multiple message indicator 
   i+= 1; 
   if (type == 1) {
      setbitu(i, 1, refd); // satellite ref datum 
      i+= 1; 
   }   
   setbitu(i, 4, iod_ssr); //iod ssr
   i += 4; 
   setbitu(i, 16, provid); //provider ID 
   i += 16; 
   setbitu(i, 4, solid); //solution ID 
   i += 4; 
   setbitu(i, 6, nsat); //no of satellites 
   i += 6; 

   return i;
}

int rtcm_ssr_encoder::epoch_to_tow(int epoch_time, int sys)
{
   time_t local_time;
   time(&local_time);

   if (sys == SYS_BEIDOU)
      local_time += 4;  //utc to bdst
   else {
      local_time += 18; //utc to gpst  
      epoch_time += 14;  //bdst to gpst
      if (epoch_time > 86400)
         epoch_time -= 86400;
   }   

   struct tm ptm;
   gmtime_r(&local_time, &ptm);

   int local_tod = ptm.tm_hour*3600 + ptm.tm_min*60 + ptm.tm_sec;

   int wday = ptm.tm_wday; 
   if (epoch_time - local_tod > 43200) {
      wday = ptm.tm_wday - 1;
      if (wday < 0)
         wday = 6;
   }
 
   return wday*3600*24 + epoch_time;
}

void rtcm_ssr_encoder::scan_satslot(bcnav3_type2 *p, int min, int max, int &num_of_sat, int &offset)
{
   num_of_sat = 0;
   offset = -1;

   for (int i = 0; i < 6; i++) {
      if (p->satslot[i] >= min && p->satslot[i] <= max) {
         if (offset == -1) {
            offset = i;
         }   
         num_of_sat++;
      }   
   }
}

//ssr1 orbit corrections
int rtcm_ssr_encoder::encode_ssr1(bcnav3_type2 *p, int sys)
{
   int nsat, off, ni, nj;
 
   if (sys == SYS_BEIDOU) {
      scan_satslot(p, 1, 63, nsat, off);
      ni = 10;
      nj = 8;
   } else if (sys == SYS_GPS) {
      scan_satslot(p, 64, 100, nsat, off);
      ni = 8;
      nj = 0;
   } else 
      return 0;

   if (nsat == 0)
      return 0;

   uint32_t tow = epoch_to_tow(p->epoch_time, sys);

   int idx = encode_ssr_head(1, sys, tow, nsat, _iodssr, 0, 0, 0);   

   for (int i = 0; i < nsat; i++) {
      int radial = ROUND(p->radial[off+i]/1E-4);
      int along = ROUND(p->along[off+i]/4E-4);
      int cross = ROUND(p->cross[off+i]/4E-4);

      int prn = sys == SYS_BEIDOU ? p->satslot[off+i] : p->satslot[off+i] - 63;     
      setbitu(idx, 6, prn); 
      idx += 6; 

      if (sys == SYS_BEIDOU) {
         setbitu(idx, ni, 0); //toe
         idx += ni; 
         setbitu(idx, nj, p->iodn[off+i]); //iodn
         idx += nj; 
      } else {
         setbitu(idx, ni, p->iodn[off+i]); 
         idx += ni; 
      }  

      setbits(idx, 22, radial); //delta radial
      idx += 22; 
      setbits(idx, 20, along); //delta along-track 
      idx += 20; 
      setbits(idx, 20, cross); //delta cross-track 
      idx += 20; 
      setbits(idx, 21, 0); //dot delta radial
      idx += 21; 
      setbits(idx, 19, 0); //dot delta along-track 
      idx += 19; 
      setbits(idx, 19, 0); //dot delta cross-track 
      idx += 19; 

      _iodcorr[p->satslot[off+i]-1] = p->iod_corr[off+i];
   }

   return idx;
}

//ssr2 clock corrections
int rtcm_ssr_encoder::encode_ssr2(bcnav3_type4 *p, int sys)
{
   if (static_cast<int>(p->iodp) != _iodp)
      return 0;

   uint32_t spos = p->sub_type*23;
   if (static_cast<std::size_t>(spos) >= _valid_sats.size())
      return 0;

   std::vector<std::array<int, 2>> clk_bias;

   for (int i = 0; i < 23 && static_cast<std::size_t>(spos+i) < _valid_sats.size(); i++) {
      int prn = _valid_sats[spos+i];
      if (sys == SYS_BEIDOU && prn > 63)
         continue;
      if (sys == SYS_GPS && prn < 64)
         continue;   
      if (p->iod_corr[i] == _iodcorr[prn-1]) {
         int c0 = ROUND(p->c0[i]/1E-4);
         if (sys == SYS_GPS)
            prn -= 63;
         clk_bias.push_back({prn, c0});
      }
   }   

   int nsat = clk_bias.size();
   if (nsat == 0)
      return 0;

   uint32_t tow = epoch_to_tow(p->epoch_time, sys);

   int idx = encode_ssr_head(2, sys, tow, nsat, _iodssr, 0, 0, 0);   

   for (int i = 0; i < nsat; i++) {
      setbitu(idx, 6, clk_bias[i][0]); //satellite ID 
      idx += 6; 
      setbits(idx, 22, clk_bias[i][1]); //delta clock C0 
      idx += 22; 
      setbits(idx, 21, 0); //delta clock C1 
      idx += 21; 
      setbits(idx, 27, 0); //delta clock C2 
      idx += 27; 
   }

   return idx;
}

//ssr3 satellite code biases
int rtcm_ssr_encoder::encdoe_ssr3(bcnav3_type3 *p, int sys)
{
   int nsat = p->sat_num;
   if (nsat == 0 || nsat > 31)
      return 0;

   uint32_t tow = epoch_to_tow(p->epoch_time, sys);

   int idx = encode_ssr_head(3, sys, tow, nsat, _iodssr, 0, 0, 0);

   for (int i = 0; i < nsat; i++) {
      setbitu(idx, 6, p->satslot[i]); // satellite ID 
      idx += 6; 

      int ncodes = p->code_bias_num[i];
      if (ncodes > 15)
         ncodes = 0;

      setbitu(idx, 5, ncodes); // number of code biases   
      idx += 5; 
        
      for (int j = 0; j < ncodes; j++) {
         int signal = p->signal[i][j] < 16 ? bds_codes[p->signal[i][j]] : 0;
         setbitu(idx, 5, signal); // signal indicator 
         idx += 5; 

         int bias = ROUND(p->code_bias[i][j]/0.01);
         setbits(idx, 14, bias); // code bias 
         idx += 14; 
      }
   }

   return idx;
}

//ssr5 ura
int rtcm_ssr_encoder::encode_ssr5(bcnav3_type2 *p, int sys)
{
   int nsat, off;
 
   if (sys == SYS_BEIDOU) {
      scan_satslot(p, 1, 63, nsat, off);
   } else if (sys == SYS_GPS) {
      scan_satslot(p, 64, 100, nsat, off);
   } else 
      return 0;
      
   if (nsat == 0)
      return 0;

   uint32_t tow = epoch_to_tow(p->epoch_time, sys);

   int idx = encode_ssr_head(5, sys, tow, nsat, _iodssr, 0, 0, 0);   

   for (int i = 0; i < nsat; i++) {
      int prn = sys == SYS_BEIDOU ? p->satslot[off+i] : p->satslot[off+i] - 63;
      setbitu(idx, 6, prn); 
      idx += 6; 
      setbitu(idx, 6, (p->ura_class[off+i] << 3) + p->ura_val[off+i]);
      idx += 6;
   }

   return idx;
}

void rtcm_ssr_encoder::update_satellites_info(std::unique_ptr<bcnav3_message> &bcnav3_msg)
{
   bcnav3_type1 *ptr = static_cast<bcnav3_type1 *>(bcnav3_msg.get());

   _iodssr = ptr->iod_ssr;

   if (static_cast<int>(ptr->iodp) != _iodp) {
      _iodp = ptr->iodp;
      _valid_sats.clear();

      uint64_t mask = 0x4000000000000000;
      for (int i = 1; i <= 63; i++) {
         if (ptr->bds_mask & mask) {
            _valid_sats.push_back(i);
         }    
         mask >>= 1;   
      }

      mask = 0x0000001000000000;  
      for (int i = 64; i <= 100; i++) {
         if (ptr->gps_mask & mask) {
            _valid_sats.push_back(i);
         }
         mask >>= 1;   
      }    
   }
}

std::vector<uint8_t> rtcm_ssr_encoder::package_rtcm_frame(int nbits)
{
   if (nbits == 0)
      return {};

   // rtcm header
   setbitu(0, 8, 0xd3); 
   setbitu(8, 6, 0); 
   setbitu(14, 10, 0); 

   // padding to align 8 bit boundary  
   int bitlen; 
   for (bitlen = nbits; bitlen%8; bitlen++) {
      setbitu(bitlen, 1, 0);
   }

   // message length (header+data) (bytes) 
   int len = bitlen/8;
   if (len >= 3+1024) 
      return {};

   // message length without header and parity 
   setbitu(14, 10, len-3);
    
   // crc-24q
   uint32_t crc = crc24q(len);
   setbitu(bitlen, 24, crc);
    
   // length total (bytes) 
   len += 3;

   return std::vector<uint8_t>(_rtcm_buf, _rtcm_buf+len);
}

std::vector<std::vector<uint8_t>> rtcm_ssr_encoder::encode(std::unique_ptr<bcnav3_message> &bcnav3_msg)
{
   LOG(INFO) << "rtcm encode : b-cnav3 type " << bcnav3_msg->msg_type << " epoch time " << bcnav3_msg->epoch_time;

   if (static_cast<int>(bcnav3_msg->iod_ssr) != _iodssr) {
      LOG(INFO) << "rtcm encode : iodssr mismatch current " << _iodssr << " input " << bcnav3_msg->iod_ssr;
      return {};
   }   

   std::vector<std::vector<uint8_t>> frames;

   if (typeid(*bcnav3_msg) == typeid(bcnav3_type2)) {
      std::vector<uint8_t> frame;
      int nbits;

      nbits = encode_ssr1(static_cast<bcnav3_type2 *>(bcnav3_msg.get()), SYS_BEIDOU);
      frame = package_rtcm_frame(nbits);
      if (frame.size())
         frames.push_back(frame);

      nbits = encode_ssr1(static_cast<bcnav3_type2 *>(bcnav3_msg.get()), SYS_GPS);
      frame = package_rtcm_frame(nbits);
      if (frame.size())
         frames.push_back(frame);   

      nbits = encode_ssr5(static_cast<bcnav3_type2 *>(bcnav3_msg.get()), SYS_BEIDOU);
      frame = package_rtcm_frame(nbits);
      if (frame.size())
         frames.push_back(frame);   

      nbits = encode_ssr5(static_cast<bcnav3_type2 *>(bcnav3_msg.get()), SYS_GPS);
      frame = package_rtcm_frame(nbits);
      if (frame.size())
         frames.push_back(frame);    
   } else if (typeid(*bcnav3_msg) == typeid(bcnav3_type3)) {
      int nbits = encdoe_ssr3(static_cast<bcnav3_type3 *>(bcnav3_msg.get()), SYS_BEIDOU);
      auto frame = package_rtcm_frame(nbits);
      if (frame.size())
         frames.push_back(frame);
   } else if(typeid(*bcnav3_msg) == typeid(bcnav3_type4)) {
      int nbits;
      std::vector<uint8_t> frame;

      nbits = encode_ssr2(static_cast<bcnav3_type4 *>(bcnav3_msg.get()), SYS_BEIDOU);
      frame = package_rtcm_frame(nbits);
      if (frame.size())
         frames.push_back(frame);

      nbits = encode_ssr2(static_cast<bcnav3_type4 *>(bcnav3_msg.get()), SYS_GPS);
      frame = package_rtcm_frame(nbits);
      if (frame.size())
         frames.push_back(frame);   
   }   

   return frames;
}

rtcm_ssr_sink_sptr make_rtcm_ssr_sink(int n_channels, int udp_port, std::string &udp_address, bool log_to_file)
{
   return rtcm_ssr_sink_sptr(new rtcm_ssr_sink(n_channels, udp_port, udp_address, log_to_file));
}

rtcm_ssr_sink::rtcm_ssr_sink(int n_channels, int udp_port, std::string &udp_address, bool log_to_file) : 
               gr::block("rtcm_ssr_sink",
               gr::io_signature::make(n_channels, n_channels, 512),
               gr::io_signature::make(0, 0, 0)),
               ioservice(), udp_socket(ioservice)
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

   remote_endpoint = ip::udp::endpoint(ip::address::from_string(d_udp_address), d_udp_port);
   udp_socket.open(boost::asio::ip::udp::v4());
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

time_t rtcm_ssr_sink::epoch_to_bdst(int epoch_time)
{
   time_t local_time;
   time(&local_time);
   local_time += 4;   //utc to bdst

   struct tm ptm;
   gmtime_r(&local_time, &ptm);

   int local_tod = ptm.tm_hour*3600 + ptm.tm_min*60 + ptm.tm_sec;

   if (epoch_time - local_tod > 43200) {
      local_time -= 3600*24;
   }   

   gmtime_r(&local_time, &ptm);
   ptm.tm_hour = 0;
   ptm.tm_min = 0;
   ptm.tm_sec = 0;

   return mktime(&ptm) + epoch_time;
}

std::string rtcm_ssr_sink::epoch_to_str(int epoch_time)
{
   time_t bdst = epoch_to_bdst(epoch_time);

   struct tm ptm;
   localtime_r(&bdst, &ptm);

   return boost::str(boost::format("%04d/%02d/%02d %02d:%02d:%02d") 
      % (1900+ptm.tm_year) % (ptm.tm_mon+1) % ptm.tm_mday % ptm.tm_hour % ptm.tm_min % ptm.tm_sec);
}

int rtcm_ssr_sink::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items, gr_vector_void_star &output_items __attribute__((unused)))
{
   const char **in = reinterpret_cast<const char **>(&input_items[0]);

   for (int channel_index = 0; channel_index < d_nchannels; channel_index++) {
      for (int item_index = 0; item_index < ninput_items[channel_index]; item_index++) {
         if (channel_index > 0)
            continue;

         std::string frame_bits(in[channel_index] + item_index*512, BEIDOU_CNAV3_FRAME_BITS);

         uint32_t prn = std::bitset<6>(frame_bits.substr(0, 6)).to_ulong();
         uint32_t ppp_status = std::bitset<6>(frame_bits.substr(6, 6)).to_ulong();
         uint32_t msg_type = std::bitset<6>(frame_bits.substr(12, 6)).to_ulong();
         uint32_t epoch_time = msg_type != 63 ? std::bitset<17>(frame_bits.substr(18, 17)).to_ulong() : 0;

         std::string time_str = msg_type != 63 ? epoch_to_str(epoch_time) : "";
         
         std::string msg_info = time_str + "  Beidou CNAV3 message  " 
            + "  prn: " + std::to_string(prn) + "  ppp status: " + std::to_string(ppp_status) 
            + "  type: " + std::to_string(msg_type);

         std::cout << msg_info << std::endl;
         if (d_log_to_file)
            d_log_files[channel_index] << msg_info << std::endl;

         auto msg_data = bcnav3_decoder.decode(frame_bits.substr(12, BEIDOU_CNAV3_DATA_LENGTH));
         if (msg_data != nullptr) {
            if (d_log_to_file) {
               d_log_files[channel_index] << msg_data << std::endl;
            }

            if (typeid(*msg_data) == typeid(bcnav3_type1))
               rtcm_encoder.update_satellites_info(msg_data);
            else {  
               auto frames = rtcm_encoder.encode(msg_data);
               if (frames.size() != 0) {
                  for (auto frame : frames) {
                     std::string byte_array;

                     for (uint8_t b : frame) {
                        byte_array += boost::str(boost::format("%02x ") % static_cast<uint32_t>(b));
                     }

                     LOG(INFO) << "rtcm message: " << byte_array;

                     udp_socket.send_to(buffer(frame.data(), frame.size()), remote_endpoint);
                  }
               }
            }      
         }
      }

      consume(channel_index, ninput_items[channel_index]);
   }

   return 0;
}
