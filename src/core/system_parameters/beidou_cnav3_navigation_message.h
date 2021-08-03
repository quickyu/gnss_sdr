#ifndef GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H
#define GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H

#include <cstdint>
#include <array>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

/*!
 * \brief This class decodes a BeiDou D1 NAV Data message
 */

constexpr int CNAV3_MSG_1 = 1;
constexpr int CNAV3_MSG_2 = 2;
constexpr int CNAV3_MSG_3 = 3;
constexpr int CNAV3_MSG_4 = 4;

class beidou_cnav3_navigation_message
{
public:
    class message1_data {
    public:    
        uint32_t epoch_time;
        uint8_t iod_ssr;
        uint8_t iodp;
        uint64_t bds_mask;
        uint64_t gps_mask;
        uint64_t gal_mask;
        uint64_t glo_mask;
    };

    class message2_data {
    public:    
        uint32_t epoch_time;
        uint8_t iod_ssr;
        std::array<uint16_t, 6> satslot;
        std::array<uint16_t, 6> iodn;
        std::array<uint8_t, 6> iod_corr;
        std::array<double, 6> radial;
        std::array<double, 6> along;
        std::array<double, 6> cross;
        std::array<uint8_t, 6> ura_class;
        std::array<uint8_t, 6> ura_val;
    };

    class message3_data {
    public:    
        uint32_t epoch_time;
        uint8_t iod_ssr;
        uint8_t sat_num;
        std::array<uint16_t, 31> sat_slot;
        std::array<uint8_t, 31> code_bias_num;
        std::array<std::array<uint8_t, 15>, 31> signal;
        std::array<std::array<double, 15>, 31> code_bias;
    };

    class message4_data {
    public:    
        uint32_t epoch_time;
        uint8_t iod_ssr;
        uint8_t iodp;
        uint8_t sub_type;
        std::array<uint8_t, 23> iod_corr;
        std::array<double, 23> c0;
    };

    beidou_cnav3_navigation_message() = default;
    ~beidou_cnav3_navigation_message() = default;

    message1_data message1_decode(std::string const &frame);
    message2_data message2_decode(std::string const &frame);
    message3_data message3_decode(std::string const &frame);
    message4_data message4_decode(std::string const &frame);

private:
    template<typename rv_type>
    rv_type read_unsigned(const std::string &data_bits, const std::pair<int32_t, int32_t> &parameter);  
};

std::ostream & operator<<(std::ostream &out, const beidou_cnav3_navigation_message::message1_data data);
std::ostream & operator<<(std::ostream &out, const beidou_cnav3_navigation_message::message2_data data);
std::ostream & operator<<(std::ostream &out, const beidou_cnav3_navigation_message::message3_data data);  
std::ostream & operator<<(std::ostream &out, const beidou_cnav3_navigation_message::message4_data data);
 
/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H
