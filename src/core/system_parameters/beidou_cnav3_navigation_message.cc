#include <boost/format.hpp>
#include <bitset>

#include "beidou_cnav3_navigation_message.h"
#include "Beidou_CNAV3.h"

template<typename rv_type>
rv_type bcnav3_message_decoder::read_unsigned(const std::string &data_bits, const std::pair<int32_t, int32_t> &parameter)
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

std::unique_ptr<bcnav3_message> bcnav3_message_decoder::decode1(std::string const &frame_data_bits)
{
    std::unique_ptr<bcnav3_type1> type1 = std::make_unique<bcnav3_type1>();

    type1->msg_type = 1;
    type1->epoch_time = read_unsigned<uint32_t>(frame_data_bits, M1_EPOCH);
    type1->iod_ssr = read_unsigned<uint8_t>(frame_data_bits, M1_IODSSR);
    type1->iodp = read_unsigned<uint8_t>(frame_data_bits, M1_IODP);
    type1->bds_mask = read_unsigned<uint64_t>(frame_data_bits, M1_BDSMASK);
    type1->gps_mask = read_unsigned<uint64_t>(frame_data_bits, M1_GPSMASK);
    type1->gal_mask = read_unsigned<uint64_t>(frame_data_bits, M1_GALMASK);
    type1->glo_mask = read_unsigned<uint64_t>(frame_data_bits, M1_GLOMASK);

    return type1;
}

std::unique_ptr<bcnav3_message> bcnav3_message_decoder::decode2(std::string const &frame_data_bits)
{
    std::unique_ptr<bcnav3_type2> type2 = std::make_unique<bcnav3_type2>();

    type2->msg_type = 2;
    type2->epoch_time = read_unsigned<uint32_t>(frame_data_bits, M2_EPOCH);
    type2->iod_ssr = read_unsigned<uint8_t>(frame_data_bits, M2_IODSSR);

    for (int i = 0; i < 6; i++) {
        type2->satslot[i] = read_unsigned<uint16_t>(frame_data_bits, std::pair<int32_t, int32_t>(M2_SATSLOT.first + 69*i, M2_SATSLOT.second));
        type2->iodn[i] = read_unsigned<uint16_t>(frame_data_bits, std::pair<int32_t, int32_t>(M2_IODN.first + 69*i, M2_IODN.second));
        type2->iod_corr[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair<int32_t, int32_t>(M2_IOD_CORR.first + 69*i, M2_IOD_CORR.second));

        int32_t val;

        val = read_unsigned<uint16_t>(frame_data_bits, std::pair<int32_t, int32_t>(M2_RADIAL.first + 69*i, M2_RADIAL.second));
        if (val >= 16384)
            val -= 32768;
        type2->radial[i] = val * 0.0016f;

        val = read_unsigned<uint16_t>(frame_data_bits, std::pair<int32_t, int32_t>(M2_ALONG.first + 69*i, M2_ALONG.second));
        if (val >= 4096)
            val -= 8192;
        type2->along[i] = val * 0.0064f;

        val = read_unsigned<uint16_t>(frame_data_bits, std::pair<int32_t, int32_t>(M2_CROSS.first + 69*i, M2_CROSS.second));   
        if (val >= 4096)
            val -= 8192;
        type2->cross[i] = val * 0.0064f;

        type2->ura_class[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair<int32_t, int32_t>(M2_URA_CLASS.first + 69*i, M2_URA_CLASS.second));
        type2->ura_val[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair<int32_t, int32_t>(M2_URA_VAL.first + 69*i, M2_URA_VAL.second));
    }

    return type2;
}

std::unique_ptr<bcnav3_message> bcnav3_message_decoder::decode3(std::string const &frame_data_bits)
{
    std::unique_ptr<bcnav3_type3> type3 = std::make_unique<bcnav3_type3>();

    type3->msg_type = 3;
    type3->epoch_time = read_unsigned<uint32_t>(frame_data_bits, M3_EPOCH);
    type3->iod_ssr = read_unsigned<uint8_t>(frame_data_bits, M3_IODSSR);
    type3->sat_num = read_unsigned<uint8_t>(frame_data_bits, M3_SATNUM);

    type3->satslot.fill({});
    type3->code_bias_num.fill({});
    type3->signal.fill({});
    type3->code_bias.fill({});

    int bit_off = M3_SATNUM.first + M3_SATNUM.second;

    for (uint32_t i = 0; i < type3->sat_num; i++) {
        type3->satslot[i] = read_unsigned<uint16_t>(frame_data_bits, std::pair<int32_t, int32_t>(bit_off, M3_SATSLOT_LEN));
        bit_off += M3_SATSLOT_LEN;
        type3->code_bias_num[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair<int32_t, int32_t>(bit_off, M3_CODEBIAS_NUM_LEN));
        bit_off += M3_CODEBIAS_NUM_LEN;
               
        for (int j = 0; j < static_cast<int>(type3->code_bias_num[i]); j++) {
            type3->signal[i][j] = read_unsigned<uint8_t>(frame_data_bits, std::pair<int32_t, int32_t>(bit_off, M3_SIGNAL_LEN));
            bit_off += M3_SIGNAL_LEN;

            int32_t val = read_unsigned<uint16_t>(frame_data_bits, std::pair<int32_t, int32_t>(bit_off, M3_CODEBIAS_LEN));
            if (val >= 2048)
                val -= 4096;
            type3->code_bias[i][j] = val * 0.017f;
            bit_off += M3_CODEBIAS_LEN;
        }
    }

    return type3;
}

std::unique_ptr<bcnav3_message> bcnav3_message_decoder::decode4(std::string const &frame_data_bits)
{
    std::unique_ptr<bcnav3_type4> type4 = std::make_unique<bcnav3_type4>();

    type4->msg_type = 4;
    type4->epoch_time = read_unsigned<uint32_t>(frame_data_bits, M4_EPOCH);
    type4->iod_ssr = read_unsigned<uint8_t>(frame_data_bits, M4_IODSSR);
    type4->iodp = read_unsigned<uint8_t>(frame_data_bits, M4_IODP);
    type4->sub_type = read_unsigned<uint8_t>(frame_data_bits, M4_SUB_TYPE);
            
    for (int i = 0; i < 23; i++) {
        type4->iod_corr[i] = read_unsigned<uint8_t>(frame_data_bits, std::pair<int32_t, int32_t>(M4_IOD_CORR.first + 18*i, M4_IOD_CORR.second));

        int32_t val = read_unsigned<uint16_t>(frame_data_bits, std::pair<int32_t, int32_t>(M4_C0.first + 18*i, M4_C0.second));
        if (val >= 16384)
            val -= 32768;
        type4->c0[i] = val * 0.0016f;
    }

    return type4;
}

std::unique_ptr<bcnav3_message> bcnav3_message_decoder::decode(std::string const &frame)
{
    uint32_t msg_type = std::bitset<6>(frame.substr(0, 6)).to_ulong();

    switch (msg_type) {
    case 1:
        return decode1(frame);
    case 2:
        return decode2(frame);
    case 3:
        return decode3(frame);
    case 4: 
        return decode4(frame);                    
    }

    return nullptr;
}

std::ostream & operator<<(std::ostream &out, std::unique_ptr<bcnav3_message> &data)
{
    if (typeid(*data) == typeid(bcnav3_type1)) {
        bcnav3_type1 *ptr = static_cast<bcnav3_type1 *>(data.get());

        out << boost::format("epoch time: %d  iod ssr: %d  iodp: %d  bds mask: %016x  "
            "gps mask: %016x  gal mask: %016x  glo mask: %016x\n")
            % ptr->epoch_time % ptr->iod_ssr % ptr->iodp % ptr->bds_mask 
            % ptr->gps_mask % ptr->gal_mask % ptr->glo_mask;

        std::vector<uint8_t> sat_index;

        uint64_t mask = 0x4000000000000000;
        for (int i = 1; i <= 63; i++) {
            if (ptr->bds_mask & mask) {
                sat_index.push_back(i);
            }    
            mask >>= 1;   
        }

        mask = 0x0000001000000000;  
        for (int i = 64; i <= 100; i++) {
            if (ptr->gps_mask & mask) {
                sat_index.push_back(i);
            }
            mask >>= 1;   
        }    

        int cnt = 0;
        for (uint8_t idx : sat_index) {
            std::string sys = idx < 64 ? "C" : "G";
            if (idx > 63) {
                idx -= 63;
            }

            out << boost::format("%s%02d ") % sys % static_cast<int>(idx);

            if (++cnt == 23) {
                cnt = 0;
                out << "\n";
            }
        }  

        out << "\n";  
    } else if (typeid(*data) == typeid(bcnav3_type2)) {
        bcnav3_type2 *ptr = static_cast<bcnav3_type2 *>(data.get());

        out << boost::format("epoch time: %d  iod ssr: %d\n") % ptr->epoch_time % ptr->iod_ssr;
    
        for (int i = 0; i < 6; i++) {
            out << boost::format("satslot: %d  iodn: %d  iod corr: %d  radial: %11.7f  along: %11.7f  "
                "cross: %11.7f  ura class: %d  ura value: %d\n")
                % static_cast<uint32_t>(ptr->satslot[i]) % static_cast<uint32_t>(ptr->iodn[i]) % static_cast<uint32_t>(ptr->iod_corr[i]) 
                % ptr->radial[i] % ptr->along[i] % ptr->cross[i]
                % static_cast<uint32_t>(ptr->ura_class[i]) % static_cast<uint32_t>(ptr->ura_val[i]);
        }
    } else if (typeid(*data) == typeid(bcnav3_type3)) {
        bcnav3_type3 *ptr = static_cast<bcnav3_type3 *>(data.get());

        out << boost::format("epoch time: %d, iod ssr: %d, sat num: %d\n") 
            % ptr->epoch_time % ptr->iod_ssr % ptr->sat_num;

        for (uint32_t i = 0; i < ptr->sat_num; i++) {
            out << boost::format("sat slot: %d  code bias num: %d  \n\t") 
                % static_cast<uint32_t>(ptr->satslot[i]) % static_cast<uint32_t>(ptr->code_bias_num[i]);

            int cnt = 0;
            for (int j = 0; j < static_cast<int>(ptr->code_bias_num[i]); j++) {
                out << boost::format("(signal: %2d  code bias: %11.7f)\t") 
                    % static_cast<uint32_t>(ptr->signal[i][j]) % ptr->code_bias[i][j];
                if (++cnt == 5) {
                    cnt = 0;
                    out << "\n\t";
                }
            }

            out << "\n";
        }

        out << "\n";
    } else if (typeid(*data) == typeid(bcnav3_type4)) {
        bcnav3_type4 *ptr = static_cast<bcnav3_type4 *>(data.get());

        out << boost::format("epoch time: %d  iod ssr: %d  iodp: %d  sub type: %d\n\t")
            % ptr->epoch_time % ptr->iod_ssr % ptr->iodp % ptr->sub_type;

        int cnt = 0;

        for (int i = 0; i < 23; i++) {
            out << boost::format("(iod corr: %d  c0: %11.7f)\t") % static_cast<uint32_t>(ptr->iod_corr[i]) % ptr->c0[i];
            if (++cnt == 5) {
                cnt = 0;
                out << "\n\t";
            }
        }   

        out << "\n"; 
    }

    return out;
}
