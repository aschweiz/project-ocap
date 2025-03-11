//
// OCAP - Open Collision Avoidance Protocol
//
// GPS data structure.
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

#ifndef __GPS_DATA_H__
#define __GPS_DATA_H__ 1

#include <inttypes.h>

typedef struct {
	uint32_t ts_sec_in_hour;	// timestamp of this data (seconds in hour)

	int32_t lat_deg_e7;     // Latitude
	int32_t lon_deg_e7;     // Longitude
	int32_t height_m;       // Height above Ellipsoid
	uint32_t hacc_cm;       // Horizontal Accuracy Estimate
	uint32_t vacc_cm;       // Vertical Accuracy Estimate
	// Not used for ADS-L:
	// int32_t vel_n_cm_s;     // NED north velocity
	// int32_t vel_e_cm_s;     // NED east velocity
	int32_t vel_u_cm_s;     // NED up! velocity
	uint32_t gspeed_cm_s;   // Ground Speed (2-D)
	int32_t heading_deg_e1; // Heading 2-D
	uint32_t sacc_cm_s;     // Speed Accuracy Estimate
	// Not used for ADS-L:
	// uint32_t cacc_deg_e1;   // Course / Heading Accuracy Estimate
} SGpsData;

#endif // __GPS_DATA_H__
