//
// CalculateOtherData.c
//
// OCAP - Open Collision Avoidance Protocol
//
// Updating the position, velocity and curve information of
// objects in our neighbourhood, based on received information.
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

#include "CalculateOtherData.h"
#include "OcapLog.h"
#include "Configuration.h"
#include "FlightPathExtrapolation.h"


// Call this method after receiving data on flight object f in our
// neighbourhood. The center z is optional.
void calculateOtherDataFromInfo(
	TFlightObjectOther *f,
	uint32_t ts,
	TVector *curPos, TVector *curVel,  			// received
	TVector *z, EOcapPathModel pathModel)   // received, optional
{
	// Here we only store the information for future processing.

	// If we have previously received a full data set, we ignore the new set.
	// TODO after a few seconds, we should use the incoming data!
	if (f->rxTs == ts && f->state == FOS_RVZ) {
		return;
	}

	// We remember the new data set.
	f->rxTs = ts;
	f->timeSinceLastRxSec = 0;
	vectorCopy(&f->rxPos, curPos);
	vectorCopy(&f->rxVel, curVel);

	// We update the state of the object.
	if (z) {
		// Full dataset available.
		ocapLogStrInt("OTHER-RVZ", 0);
		f->state = FOS_RVZ;
		vectorCopy(&f->z, z);
		f->z.x += curPos->x;
		f->z.y += curPos->y;
		f->z.z += curPos->z;
		f->pathModel = pathModel;

	} else if (f->state == FOS_UNALLOCATED) {
		// Tis shouldn't happen.
		ocapLogStrInt("OTHER-UNALLOCATED", 0);

	} else if (f->state == FOS_INIT) {
		// First set of data available.
		ocapLogStrInt("OTHER-INIT", 0);
		f->state = FOS_SINGLE_RV_PREP;

	} else if (f->state == FOS_SINGLE_RV_PREP) {
		// Shouldn't happen.
		// (The previously received data hasn't been used in a prediction.)
		ocapLogStrInt("OTHER-SINGLE-RV-PREP???", 0);

	} else if (f->state == FOS_SINGLE_RV_USE) {
		// Second set of data available.
		ocapLogStrInt("OTHER-RV-2", 0);
		f->state = FOS_RV_2;
	}
}

