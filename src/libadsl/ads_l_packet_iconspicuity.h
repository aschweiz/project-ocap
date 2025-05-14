//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L iConspicuity Packet Definition
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

#ifndef __ADS_L_PACKET_ICONSPICUITY_H__
#define __ADS_L_PACKET_ICONSPICUITY_H__ 1

#include "ads_l_packet.h"

typedef enum {
	ADSL_ICONSP_FLIGHT_STATE_UNDEF = 0,
	ADSL_ICONSP_FLIGHT_STATE_GROUND = 1,
	ADSL_ICONSP_FLIGHT_STATE_AIRBORNE = 2,
	// Reserved 3
} EAdslIConspicuityFlightState;

// ADS-L.4.SRC860.G.1 - Aircraft Category
typedef enum {
	ADSL_ICONSP_AIRCRAFT_CATEGORY_UNDEF = 0,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_LIGHT_FIXED_WING = 1,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_SMALL_TO_HEAVY_FIXED_WING = 2,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_ROTORCRAFT = 3,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_GLIDER_SAILPLANE = 4,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_LIGHTER_THAN_AIR = 5,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_ULTRALIGHT = 6,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_HANG_PARA_GLIDER = 7,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_SKYDIVER = 8,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_EVTOL_UAM = 9,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_GYROCOPTER = 10,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_UAS_OPEN_CATEGORY = 11,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_UAS_SPECIFIC_CATEGORY = 12,
	ADSL_ICONSP_AIRCRAFT_CATEGORY_UAS_CERTIFIED_CATEGORY = 13,
	// Reserved 14..31
} EAdslIConspicuityAircraftCategory;

typedef enum {
	ADSL_ICONSP_HACC_NOFIX = 0,
	ADSL_ICONSP_HACC_LT_0_5NM = 1,
	ADSL_ICONSP_HACC_LT_0_3NM = 2,
	ADSL_ICONSP_HACC_LT_0_1NM = 3,
	ADSL_ICONSP_HACC_LT_0_05NM = 4,
	ADSL_ICONSP_HACC_LT_30M = 5,
	ADSL_ICONSP_HACC_LT_10M = 6,
	ADSL_ICONSP_HACC_LT_3M = 7,
} EAdslIConspicuityHorizontalAccuracy;

typedef enum {
	ADSL_ICONSP_VACC_NOFIX = 0,
	ADSL_ICONSP_VACC_LT_150M = 1,
	ADSL_ICONSP_VACC_LT_45M = 2,
	ADSL_ICONSP_VACC_LT_15M = 3,
} EAdslIConspicuityVerticalAccuracy;

typedef enum {
	ADSL_ICONSP_VELACC_NOFIX = 0,
	ADSL_ICONSP_VELACC_LT_10MPS = 1,
	ADSL_ICONSP_VELACC_LT_3MPS = 2,
	ADSL_ICONSP_VELACC_LT_1MPS = 3,
} EAdslIConspicuityVelocityAccuracy;

typedef enum {
	ADSL_ICONSP2_PATH_MODEL_LINEAR = 0,
	ADSL_ICONSP2_PATH_MODEL_SPHERIC = 1,
	ADSL_ICONSP2_PATH_MODEL_ARC = 2,
	// Reserved 3
} EAdslIConspicuityOcapPathModel;

typedef struct {
	//
	// Header

	// - Payload type is fixed (0x02).

	// - Address mapping table (0..63)
	int addrMapEntry;

	// - Address [Manufacturer:8 BaseAddress:16]
	int addr;

	//
	// Payload (iConspicuity)

	// - Timestamp of NAV (x 250ms since full hour, %60)
	int tsSec4;

	// - Flight state
	EAdslIConspicuityFlightState flightState;

	// - Aircraft category
	EAdslIConspicuityAircraftCategory acftCategory;

	// - Emergency Status is not currently supported (0)

	// - Latitude WGS-84 (:24 signed, x 1/93206deg; 0xffffff = nofix)
	int latDeg93206;

	// - Longitude WGS-84 (:24 signed, x 1/46603; 0xffffff = nofix)
	int lonDeg46603;

	// - Ground speed (unsigned exponential encoding, 0..240m/s)
	int gndSpeed;

	// - Altitude (unsigned exponential encoding, -320m .. +61112m)
	int altMtr_scaled_2_12;

	// - Vertical speed (signed exponential encoding, -120m/s .. +120m/s)
	int vSpeed;

	// - Heading (:9, 0..511 in 0.73125deg units)
	int headingDeg07;

	// - Source integrity is not currently supported (0).
	// - Design assurance is not currently supported (0).
	// - Navigation integrity is not currently supported (0).

	// - Horizontal accuracy (7=<3m, ... 1=<.5NM, 0=>=.5NM/nofix)
	EAdslIConspicuityHorizontalAccuracy hAccuracy;

	// - Vertical accuracy (3=<15m, ... 1=<150m, 0=>=150m/nofix)
	EAdslIConspicuityVerticalAccuracy vAccuracy;

	// - Velocity accuracy
	EAdslIConspicuityVelocityAccuracy velAccuracy;

	// --------------- OCAP extension ---------------

	int useOcapExtension;

	// - Recommended model to be used by the receiver for flight path extrapolation.
	EAdslIConspicuityOcapPathModel pathModel;

	// - Vector from location to flight path center.
	// Each component is a scaled 10-bit (1+3+6) value
	// (-16192..1..16192, projected to -2024..0.125..2024).
	// The vector (in units of the velocity) points from our position to Z.
	int z[3]; // x, y, z

} SAdslIConspicuity;

#endif // __ADS_L_PACKET_ICONSPICUITY_H__
