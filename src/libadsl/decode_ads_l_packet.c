//
// OCAP - Open Collision Avoidance Protocol
//
// Decoding an ADS-L packet to GPS and configuration data.
//
// 11.07.2024 ASR  First version.
// 14.05.2025 ASR  Merged OCAP extension into IConspicuity packet structure.
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

#include "decode_ads_l_packet.h"

static int getHorizontalAccuracyCm(EAdslIConspicuityHorizontalAccuracy hacc);
static int getVerticalAccuracyCm(EAdslIConspicuityVerticalAccuracy vacc);
static int getGroundSpeedAccuracy(EAdslIConspicuityVelocityAccuracy velacc);
static int unscaleExp_2_6(int v, int isSigned);
static int unscaleExp_3_6(int v, int isSigned);
static int unscaleExp_2_12(int v, int isSigned);

// Extracts GPS and configuration information from an ADS-L packet (typically
// received from another aircraft) into GPS and configuration data structures.
// If the packet contains an OCAP extension, the corresponding information will
// be filled into pathModelOut and zV8Out, otherwise, pathModelOut will be set
// to the linear model.
void decodeAdslPacket(
  const SAdslIConspicuity *adslPacketIn,
  SGpsData *gpsDataOut, SAircraftConfig *cfgOut,
  EAdslIConspicuityOcapPathModel *pathModelOut, int zV8Out[3])
{
	// Configuration data

	cfgOut->addrMapEntry = adslPacketIn->addrMapEntry;
	cfgOut->addr = adslPacketIn->addr; // 24-bit
	cfgOut->acftCategory = adslPacketIn->acftCategory;
	cfgOut->flightState = adslPacketIn->flightState;

	// GPS data

	// Timestamp
	gpsDataOut->ts_sec_in_hour = adslPacketIn->tsSec4 >> 2; // not really sec_in_hour...

	// Position

	int64_t lat = (int64_t)(adslPacketIn->latDeg93206) * 1e7 / 93206;
	gpsDataOut->lat_deg_e7 = (int)lat;
	int64_t lon = (int64_t)(adslPacketIn->lonDeg46603) * 1e7 / 46603;
	gpsDataOut->lon_deg_e7 = (int)lon;
	gpsDataOut->height_m = unscaleExp_2_12(adslPacketIn->altMtr_scaled_2_12, 0) - 320;

	// Velocity
	gpsDataOut->gspeed_cm_s = unscaleExp_2_6(adslPacketIn->gndSpeed, 0) * 25;
	gpsDataOut->vel_u_cm_s = unscaleExp_2_6(adslPacketIn->vSpeed, 1) * 25 / 2;
	gpsDataOut->heading_deg_e1 = adslPacketIn->headingDeg07 * 225 / 32; // * (360/512) * 10

	// Accuracy
	gpsDataOut->hacc_cm = getHorizontalAccuracyCm(adslPacketIn->hAccuracy);
	gpsDataOut->vacc_cm = getVerticalAccuracyCm(adslPacketIn->vAccuracy);
	gpsDataOut->sacc_cm_s = getGroundSpeedAccuracy(adslPacketIn->velAccuracy);

	if (!adslPacketIn->useOcapExtension
		|| adslPacketIn->pathModel == ADSL_ICONSP2_PATH_MODEL_LINEAR) {
		*pathModelOut = ADSL_ICONSP2_PATH_MODEL_LINEAR;
		zV8Out[0] = 0;
		zV8Out[1] = 0;
		zV8Out[2] = 0;
		return;
	}

	// Spheric path model for r<15*v and linear for r>2024*v.
	*pathModelOut = adslPacketIn->pathModel;

	// Reconstruct the Z vector (distance from aircraft position to center of arc).
	int zxScaled = adslPacketIn->z[0];
	int zyScaled = adslPacketIn->z[1];
	int zzScaled = adslPacketIn->z[2];
	// Unscaled values are multiples of 0.125*v.
	zV8Out[0] = unscaleExp_3_6(zxScaled, 1);
	zV8Out[1] = unscaleExp_3_6(zyScaled, 1);
	zV8Out[2] = unscaleExp_3_6(zzScaled, 1);
}

static int getHorizontalAccuracyCm(EAdslIConspicuityHorizontalAccuracy hacc)
{
  switch (hacc) {
    case ADSL_ICONSP_HACC_LT_3M:
      return 300;
    case ADSL_ICONSP_HACC_LT_10M:
      return 1000; // 10m = 1000cm
    case ADSL_ICONSP_HACC_LT_30M:
      return 3000;
    case ADSL_ICONSP_HACC_LT_0_05NM:
      return 926;
    case ADSL_ICONSP_HACC_LT_0_1NM:
      return 1825;
    case ADSL_ICONSP_HACC_LT_0_3NM:
      return 5556;
    case ADSL_ICONSP_HACC_LT_0_5NM:
      return 9260;
    default:
      return 999999;
  }
}

static int getVerticalAccuracyCm(EAdslIConspicuityVerticalAccuracy vacc)
{
  switch (vacc) {
    case ADSL_ICONSP_VACC_LT_15M:
      return 1500;
    case ADSL_ICONSP_VACC_LT_45M:
      return 4500;
    case ADSL_ICONSP_VACC_LT_150M:
      return 15000;
    default:
      return 999999;
  }
}

static int getGroundSpeedAccuracy(EAdslIConspicuityVelocityAccuracy velacc)
{
  switch (velacc) {
    case ADSL_ICONSP_VELACC_LT_1MPS:
      return 100;
    case ADSL_ICONSP_VELACC_LT_3MPS:
      return 300;
    case ADSL_ICONSP_VELACC_LT_10MPS:
      return 1000;
    default:
      return 999999;
  }
}

static int unscaleExp_2_6(int v, int isSigned)
{
	int isNegative = isSigned && (v & 0x100);
	v &= 0xff;
	int e = v >> 6;
	int b = v & 0x3f;
	int result =((64 + b) << e) - 64;
	if (isNegative) {
		result = -result;
	}
	return result;
}

static int unscaleExp_3_6(int v, int isSigned)
{
	int isNegative = isSigned && (v & 0x200);
	v &= 0x1ff;
	int e = v >> 6;
	int b = v & 0x3f;
	int result =((64 + b) << e) - 64;
	if (isNegative) {
		result = -result;
	}
	return result;
}

static int unscaleExp_2_12(int v, int isSigned)
{
	int isNegative = isSigned && (v & 0x4000);
	v &= 0x3fff;
	int e = v >> 12;
	int b = v & 0xfff;
	int result =((4096 + b) << e) - 4096;
	if (isNegative) {
		result = -result;
	}
	return result;
}
