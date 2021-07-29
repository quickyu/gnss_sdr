#include <boost/crc.hpp>  
#include <boost/dynamic_bitset.hpp>
#include <boost/format.hpp>
#include <glog/logging.h>
#include <iostream>  
#include <ctime>
#include <locale>

#include "beidou_cnav3_navigation_message.h"

beidou_cnav3_navigation_message::beidou_cnav3_navigation_message()
{

}

bool beidou_cnav3_navigation_message::crc_test(const std::vector<uint8_t> &bytes, uint32_t checksum) const
{
    boost::crc_optimal<24, 0x864CFB, 0, 0, false, false> beidou_crc;

    beidou_crc.process_bytes(bytes.data(), BEIDOU_CNAV3_DATA_FRAME_BYTES);

    const uint32_t crc_computed = beidou_crc.checksum();

    return checksum == crc_computed;
}

template<typename rv_type>
rv_type beidou_cnav3_navigation_message::read_unsigned(const std::pair<int32_t, int32_t> &parameter)
{
    int start_pos = m_data_field_bits.size() - parameter.first - 1;
    rv_type result = 0;

    for (int i = 0; i < parameter.second; i++) {
        result <<= 1;
        result |= m_data_field_bits[start_pos - i];
    }
   
    return result;
}

void beidou_cnav3_navigation_message::frame_decode(std::string const &cnav_frames_str)
{
    m_data_field_bits = boost::dynamic_bitset<uint8_t>(cnav_frames_str.substr(12, 462));

    std::vector<uint8_t> data_bytes;
    boost::to_block_range(m_data_field_bits, std::back_inserter(data_bytes));
    std::reverse(data_bytes.begin(), data_bytes.end());
   
    uint32_t checksum = std::bitset<24>(cnav_frames_str.substr(474, 24)).to_ulong(); 
    d_flag_crc_test = crc_test(data_bytes, checksum); 

    if (d_flag_crc_test) {
        uint32_t prn = boost::dynamic_bitset<>(cnav_frames_str.substr(0, 6)).to_ulong();
        uint32_t ppp_status = boost::dynamic_bitset<>(cnav_frames_str.substr(6, 6)).to_ulong();
        uint32_t msg_type = read_unsigned<uint32_t>(MSGTYPE);

        std::cout << "CNAV3 message PRN=" << prn << ", ppp_status=" << ppp_status 
            << ", message_type=" << msg_type << std::endl;

        d_content.prn = prn;
        d_content.ppp_status = ppp_status;    
        d_content_update = false;

        switch (msg_type) {
        case CNAV3_MSG_1:
        {
            d_content.msg_type = msg_type;
            
            struct cnav3_msg_type_1 *msg = reinterpret_cast<struct cnav3_msg_type_1 *>(d_content.data);

            msg->epoch_time = read_unsigned<uint32_t>(M1_EPOCH);
            msg->iod_ssr = read_unsigned<uint8_t>(M1_IODSSR);
            msg->iodp = read_unsigned<uint8_t>(M1_IODP);
            msg->bds_mask = read_unsigned<uint64_t>(M1_BDSMASK);
            msg->gps_mask = read_unsigned<uint64_t>(M1_GPSMASK);
            msg->gal_mask = read_unsigned<uint64_t>(M1_GALMASK);
            msg->glo_mask = read_unsigned<uint64_t>(M1_GLOMASK);

            d_content_update = true;
            break;    
        }    
        
        case CNAV3_MSG_2:
        {
            d_content.msg_type = msg_type;

            struct cnav3_msg_type_2 *msg = reinterpret_cast<struct cnav3_msg_type_2 *>(d_content.data);

            msg->epoch_time = read_unsigned<uint32_t>(M2_EPOCH);
            msg->iod_ssr = read_unsigned<uint8_t>(M2_IODSSR);

            int step = M2_SATSLOT.second + M2_IODN.second + M2_IOD_CORR.second + M2_RADIAL.second + 
                M2_ALONG.second + M2_CROSS.second + M2_URA_CLASS.second + M2_URA_VAL.second;

            for (int i = 0; i < 6; i++) {
                msg->satslot[i] = read_unsigned<uint16_t>(std::pair(M2_SATSLOT.first + step*i, M2_SATSLOT.second));
                msg->iodn[i] = read_unsigned<uint16_t>(std::pair(M2_IODN.first + step*i, M2_IODN.second));
                msg->iod_corr[i] = read_unsigned<uint8_t>(std::pair(M2_IOD_CORR.first + step*i, M2_IOD_CORR.second));
                msg->radial[i] = read_unsigned<uint16_t>(std::pair(M2_RADIAL.first + step*i, M2_RADIAL.second));
                msg->along[i] = read_unsigned<uint16_t>(std::pair(M2_ALONG.first + step*i, M2_ALONG.second));
                msg->cross[i] = read_unsigned<uint16_t>(std::pair(M2_CROSS.first + step*i, M2_CROSS.second));
                msg->ura_class[i] = read_unsigned<uint8_t>(std::pair(M2_URA_CLASS.first + step*i, M2_URA_CLASS.second));
                msg->ura_val[i] = read_unsigned<uint8_t>(std::pair(M2_URA_VAL.first + step*i, M2_URA_VAL.second));
            }

            d_content_update = true;
            break;
        }    

        case CNAV3_MSG_3: 
        {
            d_content.msg_type = msg_type;

            struct cnav3_msg_type_3 *msg = reinterpret_cast<struct cnav3_msg_type_3 *>(d_content.data);

            msg->epoch_time = read_unsigned<uint32_t>(M3_EPOCH);
            msg->iod_ssr = read_unsigned<uint8_t>(M3_IODSSR);
            msg->sat_num = read_unsigned<uint8_t>(M3_SATNUM);

            int sat_start_pos = M3_SATSLOT.first;

            for (int i = 0; i < msg->sat_num; i++) {
                msg->sat_slot[i] = read_unsigned<uint16_t>(std::pair(sat_start_pos, M3_SATSLOT.second));
                msg->code_bias_num[i] = read_unsigned<uint8_t>(std::pair(sat_start_pos + M3_CODEBIAS_NUM_OFF.first, M3_CODEBIAS_NUM_OFF.second));
               
                int code_step = 0;

                for (int j = 0; j < msg->code_bias_num[i]; j++) {
                    msg->signal[i][j] = read_unsigned<uint8_t>(std::pair(sat_start_pos + M3_SIGNAL_OFF.first + code_step, M3_SIGNAL_OFF.second));
                    msg->code_bias[i][j] = read_unsigned<uint16_t>(std::pair(sat_start_pos + M3_CODEBIAS_OFF.first + code_step, M3_CODEBIAS_OFF.second));
                    code_step += M3_SIGNAL_OFF.second + M3_CODEBIAS_OFF.second;
                }

                sat_start_pos += M3_SATSLOT.second + M3_CODEBIAS_NUM_OFF.second + code_step;
            }

            d_content_update = true;
            break;
        }

        case CNAV3_MSG_4:
        {
            d_content.msg_type = msg_type;
            
            struct cnav3_msg_type_4 *msg = reinterpret_cast<struct cnav3_msg_type_4 *>(d_content.data);

            msg->epoch_time = read_unsigned<uint32_t>(M4_EPOCH);
            msg->iod_ssr = read_unsigned<uint8_t>(M4_IODSSR);
            msg->iodp = read_unsigned<uint8_t>(M4_IODP);
            msg->sub_type = read_unsigned<uint8_t>(M4_SUB_TYPE);
            
            for (int i = 0; i < 23; i++) {
                msg->iod_corr[i] = read_unsigned<uint8_t>(std::pair(M4_IOD_CORR.first + 18*i, M4_IOD_CORR.second));
                msg->c0[i] = read_unsigned<uint16_t>(std::pair(M4_C0.first + 18*i, M4_C0.second));
            }

            d_content_update = true;
            break;
        }   

        default: 
            std::cout << "Unknow CNAV3 message type " << msg_type << std::endl;    
        }    
    } else {
        std::cout << "CNAV3 message crc error" << std::endl;
    }
}
