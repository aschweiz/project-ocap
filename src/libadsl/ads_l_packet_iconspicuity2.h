//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L iConspicuity Packet Definition
// (Proposed extension to EASA ADS-L 4 SRD860 Issue 1)
//
// 11.10.2024 ASR  First version.
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

#ifndef __ADS_L_PACKET_ICONSPICUITY2_H__
#define __ADS_L_PACKET_ICONSPICUITY2_H__ 1

#include "ads_l_packet_iconspicuity.h"

typedef enum {
	ADSL_ICONSP2_PATH_MODEL_LINEAR = 0,
	ADSL_ICONSP2_PATH_MODEL_SPHERIC = 1,
	ADSL_ICONSP2_PATH_MODEL_ARC = 2,
	// Reserved 3
} EAdslIConspicuity2PathModel;

typedef struct {
	// Base iConspicuity packet.

	SAdslIConspicuity iconspicuity;

	// Additional payload.

	// - Recommended model to be used by the receiver for flight path extrapolation.
	EAdslIConspicuity2PathModel pathModel;

	// - Vector from location to flight path center.
	// Each component is a scaled 10-bit (1+3+6) value
	// (-16192..1..16192, projected to -2024..0.125..2024).
	// The vector (in units of the velocity) points from our position to Z.
	int z[3]; // x, y, z

} SAdslIConspicuity2;

#endif // __ADS_L_PACKET_ICONSPICUITY2_H__
