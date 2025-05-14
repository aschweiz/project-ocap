//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L iConspicuity Packet Encoding
// (Based on EASA ADS-L 4 SRD860 Issue 1)
//
// 10.07.2024 ASR  First version.
// 14.05.2025 ASR  Updated for consistency with existing implementation
//                 from Skytraxx and OGN.
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

#include <inttypes.h>
#include "ads_l_encode_iconspicuity.h"

static void encodeSigned24bit(int v, uint8_t *data3);
static uint32_t convertSignedToBinary(int v);

// Serializes the data in the provided packet for transmission, including the
// data link frame, header and payload but not including the 24-bit CRC.
// The data buffer needs to be able to hold at least 22 bytes (or 25 bytes, if
// the CRC will be added later) for standard IConspicuity packets and at least
// 30 bytes for packets with OCAP extension.
// Returns 0 on success, -1 if the data buffer is too small.
int adslEncodeIConspicuity(const SAdslIConspicuity *packet, uint8_t *data, int len)
{
	int requiredLen = packet->useOcapExtension ? 30 : 22;
	if (len < requiredLen) {
		return -1;
	}

	// 0:8, Packet length, fixed, not including itself
	// Total packet length (this field to and including CRC) is 25 bytes.
	// NOT INCLUDED in CRC check!
	data[0] = 24;

	// 8:8, Version and flags, fixed
	// From here on forward, bytes are considered for the CRC calculation.
	data[1] = 0x00;

	// --------------- Payload --------------

	// 16:8, Payload type, fixed (0x02 = iConspicuity)
	data[2] = (uint8_t)ADSL_PAYLOAD_TYPE_BROADCAST_ICONSPICUITY;

	// 24:32, Source address; LSB is relay/forward (not used)
	// From here on forward, bytes are scrambled with xxtea.
	// The bit mapping is a little bit unusual, so we use a trick here.
	uint8_t d3a = (uint8_t)(packet->addrMapEntry & 0x3f);
	uint32_t addr6 = packet->addr << 6;
	uint32_t addr6v2 = addr6 & 0x00c0c0c0;
	uint32_t addr6v6 = addr6 & 0x3f3f3f00;
	uint32_t addr6w = addr6v2 | addr6v6;
	data[3] = d3a | addr6w & 0xff;
	addr6w >>= 8;
	data[4] = addr6w & 0xff;
	addr6w >>= 8;
	data[5] = addr6w & 0xff;
	addr6w >>= 8;
	data[6] = addr6w;

	// 56:8, [FlightState:2 Timestamp:6]
	data[7] =((uint8_t)packet->flightState << 6)
		| (packet->tsSec4 & 0x3f);

	// 64:8, [Emergency(not used):3 AcftCategory:5]
	data[8] = (uint8_t)packet->acftCategory & 0x1f;

	// 72:24, Latitude in 1/93206deg units
	encodeSigned24bit(packet->latDeg93206, &data[9]);

	// 96:24, Longitude in 1/46603deg units
	encodeSigned24bit(packet->lonDeg46603, &data[12]);

	// 120:8, Ground speed (0.25m/s .. 238m/s)
	data[15] = (uint8_t)packet->gndSpeed & 0xff;

	// 128:32, [Alt:14 vSpeed:9 heading:9]
	uint32_t vspeed32 = convertSignedToBinary(packet->vSpeed) & 0x1ff;
	uint32_t heading32 = convertSignedToBinary(packet->headingDeg07) & 0x1ff;
	//      [AltH:6 AltL:8] [vSpeedH:7 vSpeedL:2] [headingH:8 headingL:1]
	// -> | [AltL:8] | [vSpeedL:2 AltH:6] | [headingL:1 vSpeedH:7] | [headingH:8] |
	data[16] = (uint8_t)(packet->altMtr_scaled_2_12 & 0xff);
	data[17] = (uint8_t)((vspeed32 << 6) & 0xc0)
		| (uint8_t)((packet->altMtr_scaled_2_12 >> 8) & 0x3f);
	data[18] = (uint8_t)((heading32 << 7) & 0x80)
		| (uint8_t)((vspeed32 >> 2) & 0x7f);
	data[19] = (uint8_t)((heading32 >> 1) & 0xff);

	// 160:8, Integrity flags (not used in this implementation)
	data[20] = 0x00;

	// 168:8, [reserved:1 velAcc:2 vAcc:2 hAcc:3]
	uint32_t velAccuracy32 = (uint32_t)packet->velAccuracy;
	uint32_t vAccuracy32 = (uint32_t)packet->vAccuracy;
	uint32_t hAccuracy32 = (uint32_t)packet->hAccuracy;
	data[21] = (uint8_t)(
		  ((velAccuracy32 << 5) & 0x60)		// velAcc:2
		| ((vAccuracy32 << 3) & 0x18)			// vAcc:2
		| (hAccuracy32 & 0x07)						// hAcc:3
	);

	// 176:24, CRC, out of scope!

	if (!packet->useOcapExtension) {
		return 0;
	}

	// --------------- OCAP extension --------------

	// We set the reserved bit at offset 119 to indicate the OCAP extension.
	data[21] |= 0x80;

	// 200:2, Path extrapolation model.
	// 202:10, Z.x
	uint32_t zx = convertSignedToBinary(packet->z[0]) & 0x3ff;
	// 212:10, Z.y
	uint32_t zy = convertSignedToBinary(packet->z[1]) & 0x3ff;
	// 222:10, Z.z
	uint32_t zz = convertSignedToBinary(packet->z[2]) & 0x3ff;
	data[25] = (((int)packet->pathModel << 6) & 0xc0)
		| (zx >> 4); // top 6 bits of Z.x
	data[26] = ((zx << 4) & 0xf0) // bottom 4 bits of Z.x
		| (zy >> 6); // top 4 bits of Z.y
	data[27] = ((zy << 2) & 0xfc) // bottom 6 bits of Z.y
		| (zz >> 8); // top 2 bits of Z.z
	data[28] = (uint8_t)(zz & 0xff); // bottom 8 bits of Zz

	// 232:8, One byte for parity checking the extension.
	data[29] = data[25] ^ data[26] ^ data[27] ^ data[28];

	return 0;
}

static void encodeSigned24bit(int v, uint8_t *data3)
{
	// Little-endian...
	uint32_t v32 = convertSignedToBinary(v);
	data3[0] = (uint8_t)(v32 & 0xff);
	data3[1] = (uint8_t)((v32 >> 8) & 0xff);
	data3[2] = (uint8_t)((v32 >> 16) & 0xff);
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
