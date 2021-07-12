#ifndef GNSS_SDR_BEIDOU_B2B_SIGNAL_REPLICA_H
#define GNSS_SDR_BEIDOU_B2B_SIGNAL_REPLICA_H

#include <complex>
#include <cstdint>
#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl/gsl>
namespace own = gsl;
#endif

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


//! Generates int BeiDou B2b code for the desired SV ID and code shift
void beidou_b2b_code_gen_int(own::span<int> dest, int32_t prn, uint32_t chip_shift);

//! Generates float BeiDou B2b code for the desired SV ID and code shift
void beidou_b2b_code_gen_float(own::span<float> dest, int32_t prn, uint32_t chip_shift);

//! Generates complex BeiDou B2b code for the desired SV ID and code shift
void beidou_b2b_code_gen_complex(own::span<std::complex<float>> dest, int32_t prn, uint32_t chip_shift);

//! Generates complex BeiDou B2b code for the desired SV ID and code shift, and sampled to specific sampling frequency
void beidou_b2b_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int sampling_freq, uint32_t chip_shift);


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B2B_SIGNAL_REPLICA_H
