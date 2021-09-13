#ifndef GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H
#define GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H

#include <cstdint>
#include <array>
#include <string>
#include <memory>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

/*!
 * \brief This class decodes a BeiDou D1 NAV Data message
 */

class bcnav3_message 
{
public:
    virtual ~bcnav3_message() = default; 

    uint32_t msg_type;
    uint32_t epoch_time;
    uint32_t iod_ssr;
};

class bcnav3_type1 : public bcnav3_message
{
public:
    uint32_t iodp;
    uint64_t bds_mask;
    uint64_t gps_mask;
    uint64_t gal_mask;
    uint64_t glo_mask;    
};

class bcnav3_type2 : public bcnav3_message
{
public:
    std::array<uint16_t, 6> satslot;
    std::array<uint16_t, 6> iodn;
    std::array<uint8_t, 6> iod_corr;
    std::array<double, 6> radial;
    std::array<double, 6> along;
    std::array<double, 6> cross;
    std::array<uint8_t, 6> ura_class;
    std::array<uint8_t, 6> ura_val;    
};

class bcnav3_type3 : public bcnav3_message
{
public:    
    uint32_t sat_num;
    std::array<uint16_t, 31> satslot;
    std::array<uint8_t, 31> code_bias_num;
    std::array<std::array<uint8_t, 15>, 31> signal;
    std::array<std::array<double, 15>, 31> code_bias;
};

class bcnav3_type4 : public bcnav3_message
{
public:
    uint32_t iodp;
    uint32_t sub_type;
    std::array<uint8_t, 23> iod_corr;
    std::array<double, 23> c0;
};

class bcnav3_message_decoder
{
private:
    template<typename rv_type>
    rv_type read_unsigned(const std::string &data_bits, const std::pair<int32_t, int32_t> &parameter);  

    std::unique_ptr<bcnav3_message> decode1(std::string const &frame_data_bits);
    std::unique_ptr<bcnav3_message> decode2(std::string const &frame_data_bits);
    std::unique_ptr<bcnav3_message> decode3(std::string const &frame_data_bits);
    std::unique_ptr<bcnav3_message> decode4(std::string const &frame_data_bits);

public:
    bcnav3_message_decoder() = default;
    ~bcnav3_message_decoder() = default;

    std::unique_ptr<bcnav3_message> decode(std::string const &frame);
};

std::ostream & operator<<(std::ostream &out, std::unique_ptr<bcnav3_message> &data);
 
/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H
