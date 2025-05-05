//
// FlightPathExtrapolation.h
//
// OCAP - Open Collision Avoidance Protocol
//
// Predicting the future flight path of an object by considering the
// object's velocity and radial component of acceleration.
//
// 02.07.2024 ASR  First version.
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

#ifndef __FLIGHT_PATH_EXTRAPOLATION_H__
#define __FLIGHT_PATH_EXTRAPOLATION_H__ 1

#include "Vector.h"

typedef enum {
	OCAP_PATH_MODEL_LINEAR = 0,
	OCAP_PATH_MODEL_SPHERIC = 1,
	OCAP_PATH_MODEL_ARC = 2,
	// Reserved 3
} EOcapPathModel;

typedef struct {
	// Input data.
	TVector r0_vec;
	TVector v0_vec;
	TVector ri_vec;
	TVector vi_vec;
	// Input data (rvz) or calculated data (2rv).
	TVector z_vec;
	// Distance r-z for iConspicuity2
	TVector r_z_vec;
	EOcapPathModel predictionModel;
	// Temporary data.
	float r;
	float v;
	float theta;
	float t_div_r;

	TVector ni_vec;
} TFlightPathExtrapolationData;


// Initialization step with 2x rv.
// It's OK to get 2 identical data points. In this case, isCurve will be set to 0.
void flightPathExtrapolationPrepare2rv(TFlightPathExtrapolationData *fpe);

// Initialization step with 1x rvz.
// Set r0/v0 and ri/vi to the same data.
// Set isCurve to 0 as input for flights on a straight line, 1 otherwise.
void flightPathExtrapolationPrepareRvz(TFlightPathExtrapolationData *fpe);


// Extrapolation step.
// Updates ri_vec and vi_vec for an 1s flight.
// If isCurve is 0, the flight will follow a straight line.
void flightPathExtrapolationExecute(TFlightPathExtrapolationData *fpe);


#endif // __FLIGHT_PATH_EXTRAPOLATION_H__
