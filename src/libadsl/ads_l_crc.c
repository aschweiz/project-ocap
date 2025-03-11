//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L CRC algorithm
// (Based on EASA ADS-L 4 SRD860 Issue 1)
//
// 11.07.2024 ASR  First version.
//
// Software License (BSD):
// Copyright 2024 Classy Code GmbH.
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

#include "ads_l_crc.h"


static uint32_t crc24(const uint8_t *data, int len_data);
static uint32_t crc24_polypass(uint32_t crc, uint8_t input);


// The CRC is calculated over 21 bytes (version+flags:1, data:20)
uint32_t adslCrc(uint8_t *data21or25, int len)
{
  return crc24(data21or25, len);
}

// See ADS-L.4.SRD860.D.1.2
static uint32_t crc24(const uint8_t *data, int len_data)
{
  uint32_t crc = 0;
  for (int idx = 0; idx < len_data; idx++) {
    crc = crc24_polypass(crc, data[idx]);
  }
  crc = crc24_polypass(crc, 0);
  crc = crc24_polypass(crc, 0);
  crc = crc24_polypass(crc, 0);
  return crc >> 8;
}

// See ADS-L.4.SRD860.D.1.2
static uint32_t crc24_polypass(uint32_t crc, uint8_t input)
{
  const uint32_t poly = 0xFFFA0480;
  crc |= input;
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (crc & 0x80000000) {
      crc ^= poly;
    }
    crc <<= 1;
  }
  return crc;
}
