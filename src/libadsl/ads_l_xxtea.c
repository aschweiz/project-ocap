//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L XXTEA algorithm
// (Based on EASA ADS-L 4 SRD860 Issue 1)
//
// 11.07.2024 ASR  First version.
// 14.05.2025 ASR  Integrated pubkey into functions for simplification.
//
// Software License (BSD):
// Copyright 2023-2025 Classy Code GmbH.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contribu-
//    tors may be used to endorse or promote products derived from this soft-
//    ware without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSE-
// QUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//

#include "ads_l_xxtea.h"

void adslXxteaEncodeWithPubkey(uint32_t *data, int lenWords)
{
    uint32_t sum = 0;
    uint32_t y = 0;
    uint32_t z = data[4];

    for (int i = 0; i < 6; i++) {
    	sum += 0x9e3779b9;
      for (int p = 0; p <= lenWords - 1; p++) {
      	y = p == 4 ? data[0] : data[p + 1];
        z = data[p] += (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + z));
      }
    }
}

void adslXxteaDecodeWithPubkey(uint32_t *data, int lenWords)
{
  uint32_t sum = 0xb54cda56;
  uint32_t y = data[0];
  uint32_t z = 0;
  for (int i = 0; i < 6; i++) {
    for (int p = lenWords - 1; p >= 0; p--) {
      z = data[(p + 4) % 5];
      y = data[p] -= (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + z));
    }
    sum -= 0x9e3779b9;
  }
}


/*
// Original algorithm as published in EASA ADS-L 4 SRD860 Issue 1, ADS-L.4.SRD860.E.2

#define MX ((((z>>5)^(y<<2))+((y>>3)^(z<<4)))^((sum^y)+(key[(p&3)^e]^z)) )

void xxtea_encrypt(
  uint32_t *data,
  uint32_t num_data_words,
  const uint32_t *key,
  uint32_t round)
{
  uint32_t z, y = data[0], sum = 0, e, DELTA = 0x9e3779b9;
  uint32_t p, q;
  z = data[num_data_words - 1];
  q = round;
  while (q-- > 0) {
    sum += DELTA;
    e = sum >> 2 & 3;
    for (p = 0; p < num_data_words - 1; p++) {
      y = data[p + 1];
      data[p] += MX;
      z = data[p];
    }
    y = data[0];
    data[num_data_words - 1] += MX;
    z = data[num_data_words - 1];
  }
}
*/
