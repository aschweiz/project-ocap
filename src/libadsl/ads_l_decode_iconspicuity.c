//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L iConspicuity Packet Decoding
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
#include "ads_l_decode_iconspicuity.h"

static int decodeSigned24bit(const uint8_t *data3);
static int convertBinaryToSigned(uint32_t v32, int nofBits);

// Decodes the data link frame, header and payload from data22 into the packet.
EAdslDecodeResult adslDecodeIConspicuity(
	const uint8_t *data, int len, SAdslIConspicuity *packet)
{
	// 0:8, Packet length
	int packetLen = data[0];
	if (packetLen < 24 || len < 22) {
		return ADSL_DECODE_INVALID_PACKET_LENGTH;
	}
	int useOcapExtension = (data[21] & 0x80) ? 1 : 0;
	if (useOcapExtension && len < 30) {
		return ADSL_DECODE_INVALID_PACKET_LENGTH;
	}
	packet->useOcapExtension = useOcapExtension;

	// 8:8, Version and flags, fixed
	// data22[1]

	// --------------- Payload --------------

	// 16:8, Payload type, fixed (0x02 = iConspicuity)
	if (data[2] != ADSL_PAYLOAD_TYPE_BROADCAST_ICONSPICUITY) {
		return ADSL_DECODE_INVALID_PACKET_TYPE;
	}

	// 24:32, Source address; Relay/forward is not used in this implementation.
	packet->addrMapEntry = data[3] & 0x3f;
	// The bit mapping is a little bit unusual, so we use a trick here.
	uint32_t addr6w =
		((uint32_t)data[6] << 24)
		| ((uint32_t)data[5] << 16)
		| ((uint32_t)data[4] << 8)
		| (uint32_t)data[3];
	uint32_t addr6v2 = (addr6w & 0x00c0c0c0) >> 6;
	uint32_t addr6v6 = (addr6w & 0x3f3f3f00) >> 6;
	uint32_t addr6 = addr6v2 | addr6v6;
	packet->addr = addr6;

	// 56:8, [FlightState:2 Timestamp:6]
	uint32_t data7 = (uint32_t)data[7];
	packet->tsSec4 = data7 & 0x3f;
	packet->flightState = (EAdslIConspicuityFlightState)(data7 >> 6);

	// 64:8, [Emergency(not used):3 AcftCategory:5]
	packet->acftCategory = (EAdslIConspicuityAircraftCategory)(data[8] & 0x1f);

	// 72:24, Latitude in 1/93206deg units, -90..90 = -8388540..8388540
	// (Nearly fills the entire 24 bits, to 0x7fffbc)
	packet->latDeg93206 = decodeSigned24bit(&data[9]);

	// 96:24, Longitude in 1/46603deg units
	packet->lonDeg46603 = decodeSigned24bit(&data[12]);

	// 120:8, Ground speed (0.25m/s .. 238m/s)
	packet->gndSpeed = data[15];

	// 128:32, [Alt:14 vSpeed:9 heading:9]
	//    | [AltL:8] | [vSpeedL:2 AltH:6] | [headingL:1 vSpeedH:7] | [headingH:8] |
	// ->   [AltH:6 AltL:8] [vSpeedH:7 vSpeedL:2] [headingH:8 headingL:1]
	uint32_t d16 = (uint32_t)data[16];
	uint32_t d17 = (uint32_t)data[17];
	uint32_t d18 = (uint32_t)data[18];
	uint32_t d19 = (uint32_t)data[19];
	packet->altMtr_scaled_2_12 = (int)(
		((d17 & 0x3f) << 8)
		| d16
	);
	uint32_t vSpeed32 =
		((d18 & 0x7f) << 2)
		| (d17 >> 6);
	packet->vSpeed = convertBinaryToSigned(vSpeed32, 9);
	uint32_t heading32 =
		(d19 << 1)
		| (d18 >> 7);
	packet->headingDeg07 = heading32;

	// 160:8, Integrity flags, not used in this implementation.
	//data[20]

	// 168:8, [reserved:1 velAcc:2 vAcc:2 hAcc:3]
	uint32_t d21 = (uint32_t)data[21];
	packet->velAccuracy = (EAdslIConspicuityVelocityAccuracy)((d21 >> 5) & 0x03);
	packet->vAccuracy = (EAdslIConspicuityVerticalAccuracy)((d21 >> 3) & 0x03);
	packet->hAccuracy = (EAdslIConspicuityHorizontalAccuracy)(d21 & 0x07);

	// 176:24, CRC, out of scope!

	if (!useOcapExtension) {
		return ADSL_DECODE_SUCCESS;
	}

	// --------------- OCAP extension --------------

	// Decode the OCAP extension.

	// 200:2, Path extrapolation model.
	// 202:10, Z.x
	// 212:10, Z.y
	// 222:10, Z.z
	uint32_t d25 = (uint32_t)data[25];
	uint32_t d26 = (uint32_t)data[26];
	uint32_t d27 = (uint32_t)data[27];
	uint32_t d28 = (uint32_t)data[28];
	packet->pathModel = (EAdslIConspicuityOcapPathModel)(d25 >> 6);
	uint32_t zxBin = ((d25 & 0x3f) << 4) | (d26 >> 4);
	uint32_t zyBin = ((d26 & 0x0f) << 6) | (d27 >> 2);
	uint32_t zzBin = ((d27 & 0x03) << 8) | d28;
	packet->z[0] = convertBinaryToSigned(zxBin, 10);
	packet->z[1] = convertBinaryToSigned(zyBin, 10);
	packet->z[2] = convertBinaryToSigned(zzBin, 10);

	// Verify the parity byte, disable the extension on mismatch.
	uint32_t ocapParity = d25 ^ d26 ^ d27 ^ d28;
	if (ocapParity != (uint32_t)data[29]) {
		packet->useOcapExtension = 0;
	}

	return ADSL_DECODE_SUCCESS;
}

static int decodeSigned24bit(const uint8_t *data3)
{
	// Little-endian...
	uint32_t v32 = ((uint32_t)data3[2] << 16)
		| ((uint32_t)data3[1] << 8)
		| data3[0];

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
