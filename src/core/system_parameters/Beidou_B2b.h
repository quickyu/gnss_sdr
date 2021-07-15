#ifndef GNSS_SDR_BEIDOU_B2B_H
#define GNSS_SDR_BEIDOU_B2B_H

#include "gnss_frequencies.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


// carrier and code frequencies
constexpr double BEIDOU_B2B_FREQ_HZ = FREQ2_BDS;          //!< BeiDou B2b [Hz]
constexpr double BEIDOU_B2B_CODE_RATE_CPS = 10.23e6;      //!< BeiDou B2b code rate [chips/s]
constexpr double BEIDOU_B2B_CODE_LENGTH_CHIPS = 10230.0;  //!< BeiDou B2b code length [chips]
constexpr double BEIDOU_B2B_CODE_PERIOD_S = 0.001;        //!< BeiDou B2b code period [seconds]
constexpr uint32_t BEIDOU_B2B_CODE_PERIOD_MS = 1;  //!< BeiDou B2b code period [ms]
constexpr int32_t BEIDOU_B2B_PREAMBLE_LENGTH_SYMBOLS = 16;
constexpr int32_t BEIDOU_B2B_TELEMETRY_SYMBOLS_PER_BIT = 1;  
constexpr char BEIDOU_B2B_PREAMBLE_SYMBOLS_STR[17] = "1110101110010000";
constexpr int32_t BEIDOU_B2B_TELEMETRY_BODY_LENGTH_SYMBOLS = 1000 - BEIDOU_B2B_PREAMBLE_LENGTH_SYMBOLS;

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B2B_H
