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

#include <inttypes.h>
#include "ads_l_encode_iconspicuity.h"

static void encodeSigned24bit(int v, uint8_t *data3);
static uint32_t convertSignedToBinary(int v);

// Serializes the data in the provided packet for transmission, including the
// data link frame, header and payload but not including the 24-bit CRC.
// The data buffer needs to be able to hold at least 22 bytes (or 25 bytes, if
// the CRC will be added later).
int adslEncodeIConspicuity(const SAdslIConspicuity *packet, uint8_t *data, int len)
{
	if (len < 22) {
		return -1;
	}

	// 0:8, Packet length, fixed, not including itself
	// Total packet length (this field to and including CRC) is 25 bytes.
	// NOT INCLUDED in CRC check!
	data[0] = 24;

	// 8:8, Version and flags, fixed
	// From here on forward, bytes are considered for the CRC calculation.
	data[1] = 0x00;

	// 16:8, Payload type, fixed (0x02 = iConspicuity)
	data[2] = (uint8_t)ADSL_PAYLOAD_TYPE_BROADCAST_ICONSPICUITY;

	// 24:32, Source address; LSB is relay/forward (not used)
	// From here on forward, bytes are scrambled with xxtea.
	data[3] = (uint8_t)(packet->addrMapEntry << 2)
		| (uint8_t)(packet->addr >> 22);
	data[4] = (uint8_t)((packet->addr >> 14) & 0xff);
	data[5] = (uint8_t)((packet->addr >> 6) & 0xff);
	data[6] = (uint8_t)((packet->addr << 2) & 0xff);

	// 56:8, [Timestamp:6 FlightState:2]
	data[7] = (uint8_t)(packet->tsSec4 << 2)
		| (uint8_t)packet->flightState;

	// 64:8, [AcftCategory:5 Emergency(not used):3]
	data[8] = (uint8_t)(packet->acftCategory << 3);

	// 72:24, Latitude in 1/93206deg units
	encodeSigned24bit(packet->latDeg93206, &data[9]);

	// 96:24, Longitude in 1/46603deg units
	encodeSigned24bit(packet->lonDeg46603, &data[12]);

	// 120:8, Ground speed (0.25m/s .. 238m/s)
	data[15] = (uint8_t)packet->gndSpeed;

	// 128:32, [Alt:14 vSpeed:9 heading:9]
	uint32_t vspeed32 = convertSignedToBinary(packet->vSpeed) & 0x1ff;
	data[16] = (uint8_t)(packet->altMtr_scaled_2_12 >> 6);
	data[17] = (uint8_t)((packet->altMtr_scaled_2_12 << 2) & 0xfc)
			| (uint8_t)((vspeed32 >> 7) & 0x03);
	uint32_t heading32 = convertSignedToBinary(packet->headingDeg07);
	data[18] = (uint8_t)((vspeed32 << 1) & 0xfe) | (uint8_t)((heading32 >> 8) & 0x01);
	data[19] = (uint8_t)(packet->headingDeg07 & 0xff);

	// 160:8, Integrity flags (not used)
	data[20] = 0x00;

	// 168:8, [hAcc:3 vAcc:2 velAcc:2 0]
	data[21] = (uint8_t)((packet->hAccuracy << 5) & 0xff)
		| (uint8_t)((packet->vAccuracy << 3) & 0xff)
		| (uint8_t)((packet->velAccuracy << 1) & 0xff);

	// 176:24, CRC, out of scope!

	return 0;
}

// Serializes the data in the provided packet for transmission, including the
// data link frame, header and payload but not including the 24-bit CRC.
// The data buffer needs to be able to hold at least 26 bytes (or 29 bytes, if
// the CRC will be added later).
// Returns 0 on success, -1 if the data buffer is too small.
int adslEncodeIConspicuity2(const SAdslIConspicuity2 *packet, uint8_t *data, int len)
{
	if (len < 26) {
		return -1;
	}

	// This fills bits 0:175 (bytes 0..21).
	adslEncodeIConspicuity(&packet->iconspicuity, data, len);

	// 0:8, Update the packet length for iConspicuity2, not including itself
	// Total packet length (this field to and including CRC) is 29 bytes.
	// NOT INCLUDED in CRC check!
	data[0] = 28;

	// 16:8, Update the payload type, fixed (0x03 = iConspicuity2)
	data[2] = (uint8_t)ADSL_PAYLOAD_TYPE_BROADCAST_ICONSPICUITY2;

	// 176:2, Path extrapolation model
	// 178:10, Z.x
	uint32_t zx = convertSignedToBinary(packet->z[0]) & 0x3ff;
	// 188:10, Z.y
	uint32_t zy = convertSignedToBinary(packet->z[1]) & 0x3ff;
	// 198:10, Z.z
	uint32_t zz = convertSignedToBinary(packet->z[2]) & 0x3ff;
	data[22] = (((int)packet->pathModel << 6) & 0xc0)
		| (zx >> 4); // top 6 bits of Z.x
	data[23] = ((zx << 4) & 0xf0) // bottom 4 bits of Z.x
		| (zy >> 6); // top 4 bits of Z.y
	data[24] = ((zy << 2) & 0xfc) // bottom 6 bits of Z.y
		| (zz >> 8); // top 2 bits of Z.z
	data[25] = (uint8_t)(zz & 0xff); // bottom 8 bits of Zz

	// 208:24, CRC, out of scope!

	return 0;
}


static void encodeSigned24bit(int v, uint8_t *data3)
{
	uint32_t v32 = convertSignedToBinary(v);
	data3[0] = (uint8_t)((v32 >> 16) & 0xff);
	data3[1] = (uint8_t)((v32 >> 8) & 0xff);
	data3[2] = (uint8_t)(v32 & 0xff);
}

static uint32_t convertSignedToBinary(int v)
{
	int isNegative = v < 0;
	if (isNegative) {
		v = -v;
	}
	uint32_t v32 = (uint32_t)v;
	if (isNegative) {
		// 2s complement
		v32 = ~v32 + 1;
	}
	return v32;
}
