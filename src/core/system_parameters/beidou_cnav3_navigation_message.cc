 #include <boost/format.hpp>

#include "beidou_cnav3_navigation_message.h"
#include "Beidou_CNAV3.h"

template<typename rv_type>
rv_type beidou_cnav3_navigation_message::read_unsigned(const std::string &data_bits, const std::pair<int32_t, int32_t> &parameter)
{
    int start_pos = parameter.first;
    rv_type result = 0;

    for (int i = 0; i < parameter.second; i++) {
        result <<= 1;

        if (data_bits[start_pos + i] == '1')
            result |= 1;
    }
   
    return result;
}

beidou_cnav3_navigation_message::message1_data beidou_cnav3_navigation_message::message1_decode(std::string const &frame_data_bits)
{
    message1_data data;

    data.epoch_time = read_unsigned<uint32_t>(frame_data_bits, M1_EPOCH);
    data.iod_ssr = read_unsigned<uint8_t>(frame_data_bits, M1_IODSSR);
    data.iodp = read_unsigned<uint8_t>(frame_data_bits, M1_IODP);
    data.bds_mask = read_unsigned<uint64_t>(frame_data_bits, M1_BDSMASK);
    data.gps_mask = read_unsigned<uint64_t>(frame_data_bits, M1_GPSMASK);
    data.gal_mask = read_unsigned<uint64_t>(frame_data_bits, M1_GALMASK);
    data.glo_mask = read_unsigned<uint64_t>(frame_data_bits, M1_GLOMASK);

    return data;
}

beidou_cnav3_navigation_message::message2_data beidou_cnav3_navigation_message::message2_decode(std::string const &frame_data_bits)
{
    message2_data data;

    data.epoch_time = read_unsigned<uint32_t>(frame_data_bits, M2_EPOCH);
    data.iod_ssr = read_unsigned<uint8_t>(frame_data_bits, M2_IODSSR);

    for (int i = 0; i < 6; i++) {
        data.satslot[i] = read_unsigned<uint16_t>(frame_data_bits, std::pair(M2_SATSLOT.first + 69*i, M2_SATSLOT.second));
        data.iodn[i] = read_unsigned<uint16_t>(frame_data_bits, std::pair(M2_IODN.first + 69*i, M2_IODN.second));
        data.iod_corr[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair(M2_IOD_CORR.first + 69*i, M2_IOD_CORR.second));

        int32_t val;

        val = read_unsigned<uint16_t>(frame_data_bits, std::pair(M2_RADIAL.first + 69*i, M2_RADIAL.second));
        if (val >= 16384)
            val -= 32768;
        data.radial[i] = val * 0.0016f;

        val = read_unsigned<uint16_t>(frame_data_bits, std::pair(M2_ALONG.first + 69*i, M2_ALONG.second));
        if (val >= 4096)
            val -= 8192;
        data.along[i] = val * 0.0064f;

        val = read_unsigned<uint16_t>(frame_data_bits, std::pair(M2_CROSS.first + 69*i, M2_CROSS.second));   
        if (val >= 4096)
            val -= 8192;
        data.cross[i] = val * 0.0064f;

        data.ura_class[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair(M2_URA_CLASS.first + 69*i, M2_URA_CLASS.second));
        data.ura_val[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair(M2_URA_VAL.first + 69*i, M2_URA_VAL.second));
    }

    return data;
}

beidou_cnav3_navigation_message::message3_data beidou_cnav3_navigation_message::message3_decode(std::string const &frame_data_bits)
{
    message3_data data;

    data.epoch_time = read_unsigned<uint32_t>(frame_data_bits, M3_EPOCH);
    data.iod_ssr = read_unsigned<uint8_t>(frame_data_bits, M3_IODSSR);
    data.sat_num = read_unsigned<uint8_t>(frame_data_bits, M3_SATNUM);

    data.sat_slot.fill({});
    data.code_bias_num.fill({});
    data.signal.fill({});
    data.code_bias.fill({});

    int32_t bit_off = M3_SATNUM.first + M3_SATNUM.second;

    for (int32_t i = 0; i < data.sat_num; i++) {
        data.sat_slot[i] = read_unsigned<uint16_t>(frame_data_bits, std::pair(bit_off, M3_SATSLOT_LEN));
        bit_off += M3_SATSLOT_LEN;
        data.code_bias_num[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair(bit_off, M3_CODEBIAS_LEN));
        bit_off += M3_CODEBIAS_LEN;
               
        for (int32_t j = 0; j < data.code_bias_num[i]; j++) {
            data.signal[i][j] = read_unsigned<uint8_t>(frame_data_bits, std::pair(bit_off, M3_SIGNAL_LEN));
            bit_off += M3_SIGNAL_LEN;

            int32_t val = read_unsigned<uint16_t>(frame_data_bits, std::pair(bit_off, M3_CODEBIAS_LEN));
            if (val >= 2048)
                val -= 4096;
            data.code_bias[i][j] = val * 0.017f;
            bit_off += M3_CODEBIAS_LEN;
        }
    }

    return data;
}

beidou_cnav3_navigation_message::message4_data beidou_cnav3_navigation_message::message4_decode(std::string const &frame_data_bits)
{
    message4_data data;

    data.epoch_time = read_unsigned<uint32_t>(frame_data_bits, M4_EPOCH);
    data.iod_ssr = read_unsigned<uint8_t>(frame_data_bits, M4_IODSSR);
    data.iodp = read_unsigned<uint8_t>(frame_data_bits, M4_IODP);
    data.sub_type = read_unsigned<uint8_t>(frame_data_bits, M4_SUB_TYPE);
            
    for (int32_t i = 0; i < 23; i++) {
        data.iod_corr[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair(M4_IOD_CORR.first + 18*i, M4_IOD_CORR.second));

        int32_t val = read_unsigned<uint16_t>(frame_data_bits, std::pair(M4_C0.first + 18*i, M4_C0.second));
        if (val >= 16384)
            val -= 32768;
        data.c0[i] = val * 0.0016f;
    }

    return data;
}

std::ostream & operator<<(std::ostream &out, const beidou_cnav3_navigation_message::message1_data data)  
{
    out << boost::format("epoch time: %d  iod ssr: %d  iodp: %d  bds mask: %016x  "
        "gps mask: %016x  gal mask: %016x  glo mask: %016x\n")
        % data.epoch_time % uint32_t(data.iod_ssr) % uint32_t(data.iodp) % data.bds_mask 
        % data.gps_mask % data.gal_mask % data.glo_mask;
    return out;
}

std::ostream & operator<<(std::ostream &out, const beidou_cnav3_navigation_message::message2_data data)
{
    out << boost::format("epoch time: %d  iod ssr: %d\n") % data.epoch_time % uint32_t(data.iod_ssr);
    
    for (int i = 0; i < 6; i++) {
        out << boost::format("satslot: %d  iodn: %d  iod corr: %d  radial: %11.7f  along: %11.7f, "
            "cross: %11.7f  ura class: %d  ura value: %d\n")
            % uint32_t(data.satslot[i]) % uint32_t(data.iodn[i]) % uint32_t(data.iod_corr[i]) 
            % data.radial[i] % data.along[i] % data.cross[i]
            % uint32_t(data.ura_class[i]) % uint32_t(data.ura_val[i]);
    }

    return out;
}

std::ostream & operator<<(std::ostream &out, const beidou_cnav3_navigation_message::message3_data data)
{
    out << boost::format("epoch time: %d, iod ssr: %d, sat num: %d\n") 
            % data.epoch_time % uint32_t(data.iod_ssr) % uint32_t(data.sat_num);

    for (int32_t i = 0; i < data.sat_num; i++) {
        out << boost::format("sat slot: %d  code bias num: %d  \n\t") % uint32_t(data.sat_slot[i]) % uint32_t(data.code_bias_num[i]);

        int cnt = 0;
        for (int32_t j = 0; j < int32_t(data.code_bias_num[i]); j++) {
            out << boost::format("(signal: %2d  code bias: %11.7f)\t") % uint32_t(data.signal[i][j]) % data.code_bias[i][j];
            if (++cnt == 5) {
                cnt = 0;
                out << "\n\t";
            }

        }

        out << "\n";
    }

    out << "\n";

    return out;
}

std::ostream & operator<<(std::ostream &out, const beidou_cnav3_navigation_message::message4_data data)
{
    out << boost::format("epoch time: %d  iod ssr: %d  iodp: %d  sub type: %d\n\t")
        % data.epoch_time % uint32_t(data.iod_ssr) % uint32_t(data.iodp) % uint32_t(data.sub_type);

    int cnt = 0;

    for (int32_t i = 0; i < 23; i++) {
        out << boost::format("(iod corr: %d  c0: %11.7f)\t") % uint32_t(data.iod_corr[i]) % data.c0[i];
        if (++cnt == 5) {
            cnt = 0;
            out <<"\n\t";
        }
    }   

    out << "\n"; 

    return out;
}

