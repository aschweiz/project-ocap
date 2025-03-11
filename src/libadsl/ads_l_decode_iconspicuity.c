//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L iConspicuity Packet Decoding
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
#include "ads_l_decode_iconspicuity.h"

static int decodeSigned24bit(uint8_t *data3);
static int convertBinaryToSigned(uint32_t v32, int nofBits);

EAdslDecodeResult adslDecodeIConspicuity(
	uint8_t *data22, SAdslIConspicuity *packet, int verifyPacketType)
{
	// 0:8, Packet length
	// data22[0]

	// 8:8, Version and flags, fixed
	// data22[1]

	// 16:8, Payload type, fixed (0x02 = iConspicuity)
	if (verifyPacketType) {
		if (data22[2] != ADSL_PAYLOAD_TYPE_BROADCAST_ICONSPICUITY) {
			return ADSL_DECODE_INVALID_PACKET_TYPE;
		}
	}

	// 24:32, Source address; LSB is relay/forward (not used)
	packet->addrMapEntry = data22[3] >> 2;
	uint32_t addr = ((uint32_t)(data22[3] & 0x03) << 22)
		| ((uint32_t)data22[4] << 14)
		| ((uint32_t)data22[5] << 6)
		| (data22[6] >> 2);
	packet->addr = addr;

	// 56:8, [Timestamp:6 FlightState:2]
	packet->tsSec4 = data22[7] >> 2;
	packet->flightState = (EAdslIConspicuityFlightState)(data22[7] & 0x03);

	// 64:8, [AcftCategory:5 Emergency(not used):3]
	packet->acftCategory = (EAdslIConspicuityAircraftCategory)(data22[8] >> 3);

	// 72:24, Latitude in 1/93206deg units, -90..90 = -8388540..8388540
	// (Nearly fills the entire 24 bits, to 0x7fffbc)
	packet->latDeg93206 = decodeSigned24bit(&data22[9]);

	// 96:24, Longitude in 1/46603deg units
	packet->lonDeg46603 = decodeSigned24bit(&data22[12]);

	// 120:8, Ground speed (0.25m/s .. 238m/s)
	packet->gndSpeed = data22[15];

	// 128:32, [Alt:14 vSpeed:9 heading:9]
	uint8_t d17 = data22[17];
	uint8_t d18 = data22[18];
	packet->altMtr_scaled_2_12 = (int)(((uint32_t)data22[16] << 6) | (d17 >> 2));
	uint32_t vSpeed32 = ((uint32_t)(d17 & 0x03) << 7) | (d18 >> 1);
	packet->vSpeed = convertBinaryToSigned(vSpeed32, 9);
	uint32_t heading32 = ((uint32_t)(d18 & 0x01) << 8) | data22[19];
	packet->headingDeg07 = heading32;

	// 160:8, Integrity flags (not used)
	//data22[20]

	// 168:8, [hAcc:3 vAcc:2 velAcc:2 0]
	uint8_t d21 = data22[21];
	packet->hAccuracy = (EAdslIConspicuityHorizontalAccuracy)(d21 >> 5);
	packet->vAccuracy = (EAdslIConspicuityVerticalAccuracy)((d21 >> 3) & 0x03);
	packet->velAccuracy = (EAdslIConspicuityVelocityAccuracy)((d21 >> 1) & 0x03);

	// 176:24, CRC, out of scope!

	return ADSL_DECODE_SUCCESS;
}

// Decodes the Data link frame, header and payload.
EAdslDecodeResult adslDecodeIConspicuity2(uint8_t *data26, SAdslIConspicuity2 *packet)
{
	// 0:8, Packet length
	// data22[0]

	// 8:8, Version and flags, fixed
	// data22[1]

	// 16:8, Payload type, fixed (0x03 = iConspicuity2)
	if (data26[2] != ADSL_PAYLOAD_TYPE_BROADCAST_ICONSPICUITY2) {
		return ADSL_DECODE_INVALID_PACKET_TYPE;
	}

	// Decode the classic iConspicuity payload.

	EAdslDecodeResult result = adslDecodeIConspicuity(data26, &packet->iconspicuity, 0);
	if (result != ADSL_DECODE_SUCCESS) {
		return result;
	}

	// Decode the extended iConspicuity2 payload.

	// 176:2, Path extrapolation model
	// 178:10, Z.x
	// 188:10, Z.y
	// 198:10, Z.z
	packet->pathModel = (EAdslIConspicuity2PathModel)(data26[22] >> 6);
	uint32_t zxBin = (uint32_t)(data26[22] & 0x3f) << 4 | (uint32_t)data26[23] >> 4;
	uint32_t zyBin = (uint32_t)(data26[23] & 0x0f) << 6 | (uint32_t)data26[24] >> 2;
	uint32_t zzBin = (uint32_t)(data26[24] & 0x03) << 8 | data26[25];
	packet->z[0] = convertBinaryToSigned(zxBin, 10);
	packet->z[1] = convertBinaryToSigned(zyBin, 10);
	packet->z[2] = convertBinaryToSigned(zzBin, 10);

	return ADSL_DECODE_SUCCESS;
}


static int decodeSigned24bit(uint8_t *data3)
{
	uint32_t v32 = ((uint32_t)data3[0] << 16)
		| ((uint32_t)data3[1] << 8)
		| data3[2];

	int v = convertBinaryToSigned(v32, 24);
	return v;
}

static int convertBinaryToSigned(uint32_t v32, int nofBits)
{
	int isNegative = v32 & (1 << (nofBits - 1));
	if (isNegative) {
		// 2s complement
		uint32_t m = 0xffffffff >> (32 - nofBits);
		v32 = ~(v32 - 1) & m;
	}
	int v = (int)v32;
	if (isNegative) {
		v = -v;
	}
	return v;
}

