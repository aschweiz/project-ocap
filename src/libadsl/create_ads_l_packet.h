//
// OCAP - Open Collision Avoidance Protocol
//
// Creating an ADS-L packet from GPS and configuration data.
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

#ifndef __CREATE_ADS_L_PACKET_H__
#define __CREATE_ADS_L_PACKET_H__ 1

#include "gps_data.h"
#include "aircraft_config.h"
#include "ads_l_packet_iconspicuity.h"
#include "ads_l_packet_iconspicuity2.h"

void createAdslPacket(SGpsData *gpsData, SAircraftConfig *cfg, SAdslIConspicuity *adsl);

// Provide Z elements as multiples of 0.125*v.
// Provide a spheric path model for r<15*v and linear for r>2024*v.
void createAdslPacket2(SGpsData *gpsData, SAircraftConfig *cfg,
  int zV8[3], EAdslIConspicuity2PathModel pathModel, SAdslIConspicuity2 *adsl);

#endif // __CREATE_ADS_L_PACKET_H__

