#ifndef GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H
#define GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H

#include "Beidou_B2b.h"
#include "Beidou_CNAV3.h"
#include <cstdint>
#include <string>
#include <vector>
#include <boost/dynamic_bitset.hpp>

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

struct cnav3_msg_type_1 {
    uint32_t epoch_time;
    uint8_t iod_ssr;
    uint8_t iodp;
    uint64_t bds_mask;
    uint64_t gps_mask;
    uint64_t gal_mask;
    uint64_t glo_mask;
};

struct cnav3_msg_type_2 {
    uint32_t epoch_time;
    uint8_t iod_ssr;
    uint16_t satslot[6];
    uint16_t iodn[6];
    uint8_t  iod_corr[6];
    uint16_t radial[6];
    uint16_t along[6];
    uint16_t cross[6];
    uint8_t ura_class[6];
    uint8_t ura_val[6];
};

struct cnav3_msg_type_3 {
    uint32_t epoch_time;
    uint8_t iod_ssr;
    uint8_t sat_num;
    uint16_t sat_slot[31];
    uint8_t code_bias_num[31];
    uint8_t signal[31][15];
    uint16_t code_bias[31][15];
};

struct cnav3_msg_type_4 {
    uint32_t epoch_time;
    uint8_t iod_ssr;
    uint8_t iodp;
    uint8_t sub_type;
    uint8_t iod_corr[23];
    uint16_t c0[23];
};

struct cnav3_message_content {
    uint8_t msg_type{0};
    uint8_t prn;
    uint8_t ppp_status;
    uint8_t data[2048];
};

class beidou_cnav3_navigation_message
{
public:
    /*!
     * Default constructor
     */
    beidou_cnav3_navigation_message();
    ~beidou_cnav3_navigation_message() = default;

    void frame_decode(std::string const &cnav_frames_str);

    friend std::ostream& operator<<(std::ostream &, beidou_cnav3_navigation_message &);

    inline bool get_flag_crc_test() const
    {
        return d_flag_crc_test;
    }

    inline bool get_update_flag() const 
    {
        return d_content_update;
    }

    inline struct cnav3_message_content & get_message()  
    {
        return d_content;
    }
 
private:
    bool crc_test(const std::vector<uint8_t> &bytes, uint32_t checksum) const;

    template<typename rv_type>
    rv_type read_unsigned(const std::pair<int32_t, int32_t> &parameter);

    boost::dynamic_bitset<uint8_t> m_data_field_bits;

    bool d_flag_crc_test{false};
    bool d_content_update{false};

    uint32_t d_crc_error_counter{0};
     
    struct cnav3_message_content d_content;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H
