//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L iConspicuity Packet Encoding
// (Based on EASA ADS-L 4 SRD860 Issue 1)
//
// 10.07.2024 ASR  First version.
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

#ifndef __ADS_L_ENCODE_ICONSPICUITY_H__
#define __ADS_L_ENCODE_ICONSPICUITY_H__ 1

#include "ads_l_packet_iconspicuity.h"
#include "ads_l_packet_iconspicuity2.h"

// Fills the Data link frame, header and payload, but not the 24-bit CRC.
// Without the CRC, the packet is 22 bytes, with CRC it's 25 bytes.
void adslEncodeIConspicuity(SAdslIConspicuity *packet, uint8_t *data22);

// Fills the Data link frame, header and payload, but not the 24-bit CRC.
// Without the CRC, the packet is 26 bytes, with CRC it's 29 bytes.
void adslEncodeIConspicuity2(SAdslIConspicuity2 *packet, uint8_t *data26);

#endif // __ADS_L_ENCODE_ICONSPICUITY_H__

