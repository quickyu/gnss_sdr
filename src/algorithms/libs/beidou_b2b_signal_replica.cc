#include "beidou_b2b_signal_replica.h"
#include <array>
#include <bitset>
#include <string>

const auto AUX_CEIL = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

void beidou_b2b_code_gen_int(own::span<int> dest, int32_t prn, uint32_t chip_shift)
{
   constexpr uint32_t code_length = 10230;
   std::bitset<code_length> G1{};
   std::bitset<code_length> G2{};
   auto G1_register = std::bitset<13>{}.set();  // All true
   auto G2_register = std::bitset<13>{}.set();  // All true
 
   bool feedback1;
   bool feedback2;
   bool aux;
   uint32_t lcv;
   uint32_t lcv2;
   uint32_t delay;
   int32_t prn_idx = prn - 1;

   const std::array<std::bitset<13>, 63> G2_register_shifted = {
         std::bitset<13>(std::string("1000000100101")), //1
         std::bitset<13>(std::string("1000000110100")), //2
         std::bitset<13>(std::string("1000010101101")), //3
         std::bitset<13>(std::string("1000101001111")), //4 
         std::bitset<13>(std::string("1000101010101")), //5
         std::bitset<13>(std::string("1000110101110")), //6
         std::bitset<13>(std::string("1000111101110")), //7
         std::bitset<13>(std::string("1000111111011")), //8
         std::bitset<13>(std::string("1001100101001")), //9
         std::bitset<13>(std::string("1001111011010")), //10
         std::bitset<13>(std::string("1010000110101")), //11
         std::bitset<13>(std::string("1010001000100")), //12
         std::bitset<13>(std::string("1010001010101")), //13
         std::bitset<13>(std::string("1010001011011")), //14
         std::bitset<13>(std::string("1010001011100")), //15
         std::bitset<13>(std::string("1010010100011")), //16
         std::bitset<13>(std::string("1010011110111")), //17
         std::bitset<13>(std::string("1010100000001")), //18
         std::bitset<13>(std::string("1010100111110")), //19
         std::bitset<13>(std::string("1010110101011")), //20
         std::bitset<13>(std::string("1010110110001")), //21
         std::bitset<13>(std::string("1011001010011")), //22
         std::bitset<13>(std::string("1011001100010")), //23
         std::bitset<13>(std::string("1011010011000")), //24
         std::bitset<13>(std::string("1011010110110")), //25
         std::bitset<13>(std::string("1011011110010")), //26
         std::bitset<13>(std::string("1011011111111")), //27
         std::bitset<13>(std::string("1011100010010")), //28
         std::bitset<13>(std::string("1011100111100")), //29
         std::bitset<13>(std::string("1011110100001")), //30
         std::bitset<13>(std::string("1011111001000")), //31
         std::bitset<13>(std::string("1011111010100")), //32
         std::bitset<13>(std::string("1011111101011")), //33
         std::bitset<13>(std::string("1011111110011")), //34
         std::bitset<13>(std::string("1100001010001")), //35
         std::bitset<13>(std::string("1100010010100")), //36
         std::bitset<13>(std::string("1100010110111")), //37
         std::bitset<13>(std::string("1100100010001")), //38
         std::bitset<13>(std::string("1100100011001")), //39
         std::bitset<13>(std::string("1100110101011")), //40
         std::bitset<13>(std::string("1100110110001")), //41
         std::bitset<13>(std::string("1100111010010")), //42
         std::bitset<13>(std::string("1101001010101")), //43
         std::bitset<13>(std::string("1101001110100")), //44
         std::bitset<13>(std::string("1101011001011")), //45
         std::bitset<13>(std::string("1101101010111")), //46
         std::bitset<13>(std::string("1110000110100")), //47
         std::bitset<13>(std::string("1110010000011")), //48
         std::bitset<13>(std::string("1110010001011")), //49
         std::bitset<13>(std::string("1110010100011")), //50
         std::bitset<13>(std::string("1110010101000")), //51
         std::bitset<13>(std::string("1110100111011")), //52
         std::bitset<13>(std::string("1110110010111")), //53
         std::bitset<13>(std::string("1111001001000")), //54
         std::bitset<13>(std::string("1111010010100")), //55
         std::bitset<13>(std::string("1111010011001")), //56
         std::bitset<13>(std::string("1111011011010")), //57
         std::bitset<13>(std::string("1111011111000")), //58
         std::bitset<13>(std::string("1111011111111")), //59
         std::bitset<13>(std::string("1111110110101")), //60
         std::bitset<13>(std::string("1111110111101")), //61 
         std::bitset<13>(std::string("0101110000101")), //62
         std::bitset<13>(std::string("0101100111011"))  //63
   };

   // A simple error check
   if ((prn_idx < 0) || (prn_idx > 62))
      return;

   // Assign shifted G2 register based on prn number
   G2_register = G2_register_shifted[prn_idx];

   // Generate G1 and G2 Register
   for (lcv = 0; lcv < code_length; lcv++) {
      G1[lcv] = G1_register[0];
      G2[lcv] = G2_register[0];

      feedback1 = G1_register[0] xor G1_register[3] xor G1_register[4] xor G1_register[12];
      feedback2 = G2_register[0] xor G2_register[1] xor G2_register[4] xor G2_register[7] xor
                  G2_register[9] xor G2_register[10];

      for (lcv2 = 0; lcv2 < 12; lcv2++) {
         G1_register[lcv2] = G1_register[lcv2 + 1];
         G2_register[lcv2] = G2_register[lcv2 + 1];
      }

      G1_register[12] = feedback1;
      G2_register[12] = feedback2;

      // Reset G1 register    
      if (lcv == 8189)   
         G1_register = std::bitset<13>{}.set();  // All true
   }

   delay = code_length;
   delay += chip_shift;
   delay %= code_length;

   // Generate PRN from G1 and G2 Registers
   for (lcv = 0; lcv < code_length; lcv++) {
      aux = G1[(lcv + chip_shift) % code_length] xor G2[delay];
      if (aux == true) {
         dest[lcv] = 1;
      } else {
         dest[lcv] = -1;
      }

      delay++;
      delay %= code_length;
   }
}

void beidou_b2b_code_gen_float(own::span<float> dest, int32_t prn, uint32_t chip_shift)
{
   constexpr uint32_t code_length = 10230;
   std::array<int, code_length> b2b_code_int{};

   beidou_b2b_code_gen_int(b2b_code_int, prn, chip_shift);

   for (uint32_t ii = 0; ii < code_length; ++ii) {
      dest[ii] = static_cast<float>(b2b_code_int[ii]);
   }
}

void beidou_b2b_code_gen_complex(own::span<std::complex<float>> dest, int32_t prn, uint32_t chip_shift)
{
   constexpr uint32_t code_length = 10230;
   std::array<int, code_length> b2b_code_int{};

   beidou_b2b_code_gen_int(b2b_code_int, prn, chip_shift);

   for (uint32_t ii = 0; ii < code_length; ++ii) {
      dest[ii] = std::complex<float>(static_cast<float>(b2b_code_int[ii]), 0.0F);
   }
}

void beidou_b2b_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int sampling_freq, uint32_t chip_shift)
{
   constexpr int32_t codeFreqBasis = 10230000;  // chips per second
   constexpr int32_t codeLength = 10230;
   constexpr float tc = 1.0 / static_cast<float>(codeFreqBasis);  // B2b chip period in sec

   const float ts = 1.0F / static_cast<float>(sampling_freq);  // Sampling period in secs
   const auto samplesPerCode = static_cast<int32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / static_cast<double>(codeLength)));

   std::array<std::complex<float>, 10230> code_aux{};

   int32_t codeValueIndex;
   float aux;

   beidou_b2b_code_gen_complex(code_aux, prn, chip_shift);  // generate B2b code 1 sample per chip

    for (int32_t i = 0; i < samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read B2b code values --------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one B2b code period is
            // one millisecond).

            aux = (ts * (static_cast<float>(i) + 1)) / tc;
            codeValueIndex = AUX_CEIL(aux) - 1;

            // --- Make the digitized version of the B2b code ------------------
            // The upsampled code is made by selecting values from the B2b code
            // chip array for the time instances of each sample.
            if (i == samplesPerCode - 1)
                {
                    // Correct the last index (due to number rounding issues)
                    dest[i] = code_aux[codeLength - 1];
                }
            else
                {
                    dest[i] = code_aux[codeValueIndex];  // repeat the chip -> upsample
                }
        }
}
