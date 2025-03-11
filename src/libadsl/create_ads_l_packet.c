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

#include <math.h>
#include "create_ads_l_packet.h"

static EAdslIConspicuityHorizontalAccuracy getHorizontalAccuracy(int haccCm);
static EAdslIConspicuityVerticalAccuracy getVerticalAccuracy(int vaccCm);
static EAdslIConspicuityVelocityAccuracy getGroundSpeedAccuracy(int speedCm);
static int scaleExp_2_6(int v, int isSigned);
static int scaleExp_3_6(int v, int isSigned);
static int scaleExp_2_12(int v, int isSigned);

void createAdslPacket(SGpsData *gpsData, SAircraftConfig *cfg, SAdslIConspicuity *adsl)
{
  // Configuration data

	adsl->addrMapEntry = cfg->addrMapEntry;
	adsl->addr = cfg->addr; // 24-bit
	adsl->flightState = cfg->flightState;
	adsl->acftCategory = cfg->acftCategory;

	// GPS data

	// Timestamp
	adsl->tsSec4 = (gpsData->ts_sec_in_hour << 2) % 60; // 0..59s4, 0..15s

	// Position
	int64_t lat = (int64_t)(gpsData->lat_deg_e7) * 93206 / 1e7;
	adsl->latDeg93206 = (int)lat; // 4427285 = 47.5N
	int64_t lon = (int64_t)(gpsData->lon_deg_e7) * 46603 / 1e7;
	adsl->lonDeg46603 = (int)lon; // -396126 = 8.5W
	adsl->altMtr_scaled_2_12 = scaleExp_2_12(gpsData->height_m + 320, 0); // 0x0528 = 1000m

	// Velocity
	adsl->gndSpeed = scaleExp_2_6(gpsData->gspeed_cm_s / 25, 0); // 0xc4 = 120m/s
	int velU = gpsData->vel_u_cm_s * 2;
	// Divide with "rounding" to next integer.
	velU = velU >= 0 ? (velU + 12) : (velU - 12);
	velU /= 25;
	adsl->vSpeed = scaleExp_2_6(velU, 1);
	adsl->headingDeg07 = gpsData->heading_deg_e1 * 32 / 225; // / 10 * (512/360)

	// Accuracy
	adsl->hAccuracy = getHorizontalAccuracy(gpsData->hacc_cm);
	adsl->vAccuracy = getVerticalAccuracy(gpsData->vacc_cm);
	adsl->velAccuracy = getGroundSpeedAccuracy(gpsData->sacc_cm_s);
}

void createAdslPacket2(SGpsData *gpsData, SAircraftConfig *cfg,
	int zV8[3], EAdslIConspicuity2PathModel pathModel, SAdslIConspicuity2 *adsl)
{
	createAdslPacket(gpsData, cfg, &adsl->iconspicuity);

	adsl->pathModel = pathModel;
	adsl->z[0] = scaleExp_3_6(zV8[0], 1);
	adsl->z[1] = scaleExp_3_6(zV8[1], 1);
	adsl->z[2] = scaleExp_3_6(zV8[2], 1);
}

static EAdslIConspicuityHorizontalAccuracy getHorizontalAccuracy(int haccCm)
{
	if (haccCm < 300) { // cm
		return ADSL_ICONSP_HACC_LT_3M;
	} else if (haccCm < 1000) {
		return ADSL_ICONSP_HACC_LT_10M;
	} else if (haccCm < 3000) {
		return ADSL_ICONSP_HACC_LT_30M;
// These can never happen:
//	} else if (haccCm < 926) {
//		return ADSL_ICONSP_HACC_LT_0_05NM;
//	} else if (haccCm < 1825) {
//		return ADSL_ICONSP_HACC_LT_0_1NM;
	} else if (haccCm < 5556) {
		return ADSL_ICONSP_HACC_LT_0_3NM;
	} else if (haccCm < 9260) {
		return ADSL_ICONSP_HACC_LT_0_5NM;
	}
	return ADSL_ICONSP_HACC_NOFIX;
}

static EAdslIConspicuityVerticalAccuracy getVerticalAccuracy(int vaccCm)
{
	if (vaccCm < 1500) {
		return ADSL_ICONSP_VACC_LT_15M;
	} else if (vaccCm < 4500) {
		return ADSL_ICONSP_VACC_LT_45M;
	} else if (vaccCm < 15000) {
		return ADSL_ICONSP_VACC_LT_150M;
	}
	return ADSL_ICONSP_VACC_NOFIX;
}

static EAdslIConspicuityVelocityAccuracy getGroundSpeedAccuracy(int speedCm)
{
	if (speedCm < 100) {
		return ADSL_ICONSP_VELACC_LT_1MPS;
	} else if (speedCm < 300) {
		return ADSL_ICONSP_VELACC_LT_3MPS;
	} else if (speedCm < 1000) {
		return ADSL_ICONSP_VELACC_LT_10MPS;
	}
	return ADSL_ICONSP_VELACC_NOFIX;
}

static int scaleExp_2_6(int v, int isSigned)
{
	int isNegative = 0;
	if (isSigned && v < 0) {
		isNegative = 1;
		v = -v;
	}
	int e = 0;
	int b = 0;
	if (v >= 448) {
		e = 3;
		b = (v - 448) >> 3;
	} else if (v >= 192) {
		e = 2;
		b = (v - 192) >> 2;
	} else if (v >= 64) {
		e = 1;
		b = (v - 64) >> 1;
	} else {
		e = 0;
		b = v;
	}
	return (e << 6) | b | (isNegative ? 0x100 : 0);
}

static int scaleExp_3_6(int v, int isSigned)
{
	int isNegative = 0;
	if (isSigned && v < 0) {
		isNegative = 1;
		v = -v;
	}
	// TODO MED put this in a for loop, make it more generic
	int e = 0;
	int b = 0;
	if (v >= (64 + 128 + 256 + 512 + 1024 + 2048 + 4096)) {
		e = 7;
		b = (v - (64 + 128 + 256 + 512 + 1024 + 2048 + 4096)) >> 7;
	} else if (v >= (64 + 128 + 256 + 512 + 1024 + 2048)) {
		e = 6;
		b = (v - (64 + 128 + 256 + 512 + 1024 + 2048)) >> 6;
	} else if (v >= (64 + 128 + 256 + 512 + 1024)) {
		e = 5;
		b = (v - (64 + 128 + 256 + 512 + 1024)) >> 5;
	} else if (v >= (64 + 128 + 256 + 512)) {
		e = 4;
		b = (v - (64 + 128 + 256 + 512)) >> 4;
	} else if (v >= (64 + 128 + 256)) {
		e = 3;
		b = (v - (64 + 128 + 256)) >> 3;
	} else if (v >= (64 + 128)) {
		e = 2;
		b = (v - (64 + 128)) >> 2;
	} else if (v >= 64) {
		e = 1;
		b = (v - 64) >> 1;
	} else {
		e = 0;
		b = v;
	}
	return (e << 6) | b | (isNegative ? 0x200 : 0);
}

static int scaleExp_2_12(int v, int isSigned)
{
	int isNegative = 0;
	if (isSigned && v < 0) {
		isNegative = 1;
		v = -v;
	}
	int e = 0;
	int b = 0;
	if (v >= 28672) {
		e = 3;
		b = (v - 28672) >> 3;
	} else if (v >= 12288) {
		e = 2;
		b = (v - 12288) >> 2;
	} else if (v >= 4096) {
		e = 1;
		b = (v - 4096) >> 1;
	} else {
		e = 0;
		b = v;
	}
	return (e << 12) | b | (isNegative ? 0x4000 : 0);
}
