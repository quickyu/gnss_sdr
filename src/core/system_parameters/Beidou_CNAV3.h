#ifndef GNSS_SDR_BEIDOU_CNAV3_H
#define GNSS_SDR_BEIDOU_CNAV3_H

#include <cstdint>
#include <utility>
#include <vector>

constexpr int32_t BEIDOU_CNAV3_PREAMBLE_SYMBOLS = 16;
constexpr int32_t BEIDOU_CNAV3_TELEMETRY_MESSAGE_SYMBOLS = 1000;
constexpr int32_t BEIDOU_CNAV3_DATA_FRAME_BITS = 462;
constexpr int32_t BEIDOU_CNAV3_DATA_FRAME_BYTES = 58;
constexpr char BEIDOU_CNAV3_PREAMBLE[17] = "1110101110010000";

const std::pair<int32_t, int32_t> MSGTYPE{0, 6};

const std::pair<int32_t, int32_t> M1_EPOCH{6, 17};
const std::pair<int32_t, int32_t> M1_IODSSR{27, 2};
const std::pair<int32_t, int32_t> M1_IODP{29, 4};
const std::pair<int32_t, int32_t> M1_BDSMASK{33, 63};
const std::pair<int32_t, int32_t> M1_GPSMASK{96, 37};
const std::pair<int32_t, int32_t> M1_GALMASK{133, 37};
const std::pair<int32_t, int32_t> M1_GLOMASK{170, 37};

const std::pair<int32_t, int32_t> M2_EPOCH{6, 17};
const std::pair<int32_t, int32_t> M2_IODSSR{27, 2};
const std::pair<int32_t, int32_t> M2_SATSLOT{29, 9};
const std::pair<int32_t, int32_t> M2_IODN{38, 10};
const std::pair<int32_t, int32_t> M2_IOD_CORR{48, 3};
const std::pair<int32_t, int32_t> M2_RADIAL{51, 15};
const std::pair<int32_t, int32_t> M2_ALONG{66, 13};
const std::pair<int32_t, int32_t> M2_CROSS{79, 13};
const std::pair<int32_t, int32_t> M2_URA_CLASS{92, 3};
const std::pair<int32_t, int32_t> M2_URA_VAL{95, 3};

const std::pair<int32_t, int32_t> M3_EPOCH{6, 17};
const std::pair<int32_t, int32_t> M3_IODSSR{27, 2};
const std::pair<int32_t, int32_t> M3_SATNUM{29, 5};
const std::pair<int32_t, int32_t> M3_SATSLOT{34, 9};
const std::pair<int32_t, int32_t> M3_CODEBIAS_NUM_OFF{9, 4};
const std::pair<int32_t, int32_t> M3_SIGNAL_OFF{13, 4};
const std::pair<int32_t, int32_t> M3_CODEBIAS_OFF{17, 12};

const std::pair<int32_t, int32_t> M4_EPOCH{6, 17};
const std::pair<int32_t, int32_t> M4_IODSSR{27, 2};
const std::pair<int32_t, int32_t> M4_IODP{29, 4};
const std::pair<int32_t, int32_t> M4_SUB_TYPE{33, 5};
const std::pair<int32_t, int32_t> M4_IOD_CORR{38, 3};
const std::pair<int32_t, int32_t> M4_C0{41, 15};

#endif