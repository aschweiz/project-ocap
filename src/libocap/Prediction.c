//
// Prediction.c
//
// OCAP - Open Collision Avoidance Protocol
//
// Algorithm for detecting potential collisions.
//
// 02.07.2024 ASR  First version.
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

#include <stddef.h>
#include "Configuration.h"
#include "FlightObjectOwn.h"
#include "FlightObjectOther.h"
#include "FlightObjectList.h"
#include "FlightPathExtrapolation.h"
#include "Prediction.h"
#include "AlarmStateList.h"
#include "OcapLog.h"


// Made this globally visible for the simulation
/* static */ TVector sOwnFlightPath[T_MAX_SEC];
static float sOwnVelMsSqu[T_MAX_SEC];


// The simulation code can inject a pointer of FLIGHT_OBJECT_LIST_LENGTH
// TVector arrays of length T_MAX_SEC here.
TVector *sOtherFlightPath;

// The simulation code can inject a pointer of FLIGHT_OBJECT_LIST_LENGTH
// arrays of uint32_t here. It needs to clear this array before each
// prediction and can retrieve the idNr of the aircraft after the
// prediction.
uint32_t *sOtherFlightPathIdNr;

static float sK = 30;
static float sFacLevel2 = 2;
static float sFacLevel1 = 3;

static TFlightPathExtrapolationData sFpe;


static void predictionExtrapolateOwnFlightPath(
	TFlightObjectOwn *f);

static void predictionPrepareExtrapolationOtherFlightPath(
	TFlightObjectOther *f, uint32_t ts);

static TAlarmState *predictionCalculateAlarmStateForFlightObject(
	int t,
	TVector *posSelf, float vOwnMsSqu,
	TFlightObjectOther *f, TVector *posF, float vFmsSqu);

void predictionInit(float k, float facLevel2, float facLevel1)
{
	sK = k;
	sFacLevel2 = facLevel2;
	sFacLevel1 = facLevel1;
}

void predictionCalculateAlarmStates(uint32_t ts)
{
	// Start with no alarms.
	alarmStateListClear();

	// Extrapolate our own flight path.
	TFlightObjectOwn *fOwn = flightObjectListGetOwn();
	ocapLogFlOwn(fOwn);

	predictionExtrapolateOwnFlightPath(fOwn);

	ocapLogFlOwnPath(&sOwnFlightPath[0]);

	// Loop through all objects in the neighbourhood to detect potential collisions.
	int nofFlightObjectsOther = flightObjectListGetOtherCount();
	for (int i = 0; i < nofFlightObjectsOther; i++) {

		TFlightObjectOther *fOther = flightObjectListGetOtherAtIndex(i);

		// Skip flight objects for which we have no valid data.
		if (fOther->state == FOS_INIT || fOther->state == FOS_UNALLOCATED) {
			ocapLogStrInt("FLOBJ-UNINIT", i);
			continue;
		}

		// If we have received data, we need to apply it to the flight object.
		predictionPrepareExtrapolationOtherFlightPath(fOther, ts);

		// Square of the other aircraft's velocity.
		float vOtherMsSqu =
			fOther->vel_i0.x * fOther->vel_i0.x
			+ fOther->vel_i0.y * fOther->vel_i0.y
			+ fOther->vel_i0.z * fOther->vel_i0.z;

		// Extrapolate into the future and check for potential collisions.
		// We remember the current distance (at t=0) to the other aircraft
		// in the alarm state, if an alarm state is generated.
		TAlarmState *alarmStateForOther = NULL;
		TVector distToOther;
		for (int t = 0; t < T_MAX_SEC; t += T_DELTA_SEC) {

			flightPathExtrapolationExecute(&sFpe);

			// Compare our position to that of the other object.
			TVector *ownPos = &sOwnFlightPath[t];
			float ownVelMsSqu = sOwnVelMsSqu[t];
			TVector *otherPos = &sFpe.ri_vec;

			// Remember the distance at t=0 to store it in a potential alarm state.
			if (t == 0) {
				vectorCopy(&distToOther, otherPos);
				vectorSubtractVector(&distToOther, ownPos);
			}

			// Provide predicted flight paths to the simulation code.
			if (sOtherFlightPath && sOtherFlightPathIdNr) {
				int offset = i * T_MAX_SEC + t;
				sOtherFlightPath[offset].x = otherPos->x;
				sOtherFlightPath[offset].y = otherPos->y;
				sOtherFlightPath[offset].z = otherPos->z;
				sOtherFlightPathIdNr[i] = fOther->id;
			}

			if (t < 4) {
				ocapLogFlOtherPath1(otherPos, t);
			}

			TAlarmState *newAlarmState = predictionCalculateAlarmStateForFlightObject(
				t, ownPos, ownVelMsSqu, fOther, otherPos, vOtherMsSqu);
			if (newAlarmState) {
				alarmStateForOther = newAlarmState;
			}
		}

		if (alarmStateForOther) {
			vectorCopy(&alarmStateForOther->curDistanceToFlightObject, &distToOther);
		}
	}

	// Release the flight objects for which we didn't receive data for a long time.
	for (int i = 0; i < nofFlightObjectsOther; i++) {
		TFlightObjectOther *f = flightObjectListGetOtherAtIndex(i);
		if (f->state != FOS_UNALLOCATED) {
			f->timeSinceLastRxSec++;
			if (f->timeSinceLastRxSec >= FLIGHT_OBJECT_TIME_TO_RELEASE_SEC) {
				ocapLogStrInt("FLOBJ-RELEASE", f->id);
				flightObjectListRemoveOtherAtIndex(i);
			}
		}
	}
}

static void predictionExtrapolateOwnFlightPath(
	TFlightObjectOwn *f)
{
	// TODO HIGH what if we didn't receive GPS info?

	// Mark the object as "fresh".
	f->timeSinceLastRxSec = 0;

	// Prepare the calculation parameters.

	// TODO HIGH
	if (f->pos_vel_i_ctr == 0) {
		return;
	}

	vectorCopy(&sFpe.r0_vec, &f->pos_i[f->pos_vel_i_ctr - 1]);
	vectorCopy(&sFpe.v0_vec, &f->vel_i[f->pos_vel_i_ctr - 1]);
	vectorCopy(&sFpe.ri_vec, &f->rxPos);
	vectorCopy(&sFpe.vi_vec, &f->rxVel);

	// $TEST,1721390405.084,0,FL-OWN-FPE,745207,5256480,745207,5256496
	ocapLogFpe(1, &sFpe);

	flightPathExtrapolationPrepare2rv(&sFpe);

	// $TEST,1721390405.090,0,FL-OWN-FPE,1,2147483647,2147483647,2147483647,20
	ocapLogRZxyV(1, &sFpe);

	// Store our own Z and prediction model for the iConspicuity2 transmission.
	flightObjectOwnActivateModel(f, &sFpe.z_vec, sFpe.predictionModel);

	// For all future points in time t ...
	for (int t = 0; t < T_MAX_SEC; t += T_DELTA_SEC) {

		// ... predict the position and velocity for our aircraft ...
		flightPathExtrapolationExecute(&sFpe);

		// ... and save the position and velocity for later.
		vectorCopy(&sOwnFlightPath[t], &sFpe.ri_vec);
		float vSqu = sFpe.vi_vec.x * sFpe.vi_vec.x
			+ sFpe.vi_vec.y * sFpe.vi_vec.y
			+ sFpe.vi_vec.z * sFpe.vi_vec.z;
		sOwnVelMsSqu[t] = vSqu;
	}
}

static void predictionPrepareExtrapolationOtherFlightPath(
	TFlightObjectOther *f, uint32_t ts)
{
	// Different scenarioes:
	// - We received a packet in this iteration for this aircraft:
	//   - Full data (r/v/z) available:  Full extrapolation/prediction.
	//   - Partial data available:       Update z; full extrapolation/prediction.
	// - No packet received:
	//   - Old data (r/v) available:     Linear e/p; extrapolate "now"
	//   - Old data (r/v/z) available:   Full e/p; extrapolate "now"
	//   - No data available:            skip

	// Special case: 1 data point, not used before.
	// (This only happens if this is a new data point.)
	if (f->state == FOS_SINGLE_RV_PREP) {
		// We can't calculate Z; we use linear extrapolation.
		// r/v is not set, needs to be set first
		ocapLogStrInt("OTHER-SINGLE-RV-USE", 0);
		flightObjectOtherActivateRxData(f);
		f->state = FOS_SINGLE_RV_USE;
	}

	vectorCopy(&sFpe.r0_vec, &f->pos_i0);
	vectorCopy(&sFpe.v0_vec, &f->vel_i0);
	vectorCopy(&sFpe.ri_vec, &f->rxPos);
	vectorCopy(&sFpe.vi_vec, &f->rxVel);

	// $TEST,1721390405.084,0,FL-OTHER-FPE,745207,5256480,745207,5256496
	ocapLogFpe(0, &sFpe);

	if (f->state == FOS_RVZ) {
		// We have Z.
		vectorCopy(&sFpe.z_vec, &f->z);
		sFpe.predictionModel = f->pathModel;
		flightPathExtrapolationPrepareRvz(&sFpe);

	} else if (f->state == FOS_RV_2) {
		// We can calculate Z.
		flightPathExtrapolationPrepare2rv(&sFpe);
		vectorCopy(&f->z, &sFpe.z_vec);

	} else if (f->state == FOS_SINGLE_RV_USE) {
		// We can't calculate Z; we use linear extrapolation.
		// We have two (identical) points for the calculation.
		flightPathExtrapolationPrepare2rv(&sFpe);
	}

	ocapLogRZxyV(0, &sFpe);

	ocapLogFlOtherTs(f->rxTs, ts);

	// If the data has been received in this iteration, this is it.
	int32_t deltaSec = (int32_t)(ts - f->rxTs);
	if (deltaSec < 0
				// Avoid too many extrapolation steps in case of a problem.
				|| deltaSec > 3) {
		ocapLogStrInt("ACTI_<=0_>3", (int)deltaSec);
		// Activate the new data.
		flightObjectOtherActivateRxData(f);
		return;
	}

	// The rx data hasn't been received in this iteration.
	// We run extrapolation steps to get the current position and velocity.

	while (deltaSec > 0) {
		ocapLogStrInt("EXTRAP", deltaSec);
		flightPathExtrapolationExecute(&sFpe);
		deltaSec--;
	}

	// Copy it back for the future.
	vectorCopy(&f->pos_i0, &sFpe.ri_vec);
	vectorCopy(&f->vel_i0, &sFpe.vi_vec);
	f->ts = ts;
}

static TAlarmState *predictionCalculateAlarmStateForFlightObject(
	int t,
	TVector *posSelf, float vOwnMsSqu,
	TFlightObjectOther *f, TVector *posF, float vFmsSqu)
{
	float dx = posF->x - posSelf->x;
	float dy = posF->y - posSelf->y;
	float dz = posF->z - posSelf->z;

	// To save performance and avoid sqrt, we work with the squared distance.
	float distMtrSqu = dx*dx + dy*dy + dz*dz;

	// ocapLogStrInt("OCAP-DIST-M2", (int)distMtrSqu);

	// Only check if within the limit.
	if (distMtrSqu > DIST_MTR_SQU_CHECK_LIMIT) {
		return NULL;
	}

	uint32_t distMtrSquInt = (uint32_t)distMtrSqu;
	EAlarmLevel newAlarmLevel = ALARM_LEVEL_NONE;

	// Provide a lower bound (fixed safety zone) of 5m for each aircraft.
	// TODO MED make this configurable, or dependent on the aircraft?
	if (vOwnMsSqu < 25.0f) {
		vOwnMsSqu = 25.0f;
	}
	if (vFmsSqu < 25.0f) {
		vFmsSqu = 25.0f;
	}

	// Check if the predicted position of the other flight object
	// is in one of our 3 per-level truncated cones.
	float dMinL3 = (1.0f + 1.0f/sK * t) * sqrtf(vOwnMsSqu + vFmsSqu);
	// Compensate Z vector resolution in ADS-L.
	dMinL3 += 0.001f * ALARM_Z_OTHER_COMPENSATION_MILLI_V * sqrtf(vFmsSqu);
	// Compensate input vector noise.
	dMinL3 += 0.001f * ALARM_Z_OWN_COMPENSATION_MILLI_V * sqrtf(vOwnMsSqu);
	uint32_t dMinL3SquInt = (uint32_t)(dMinL3 * dMinL3);
	uint32_t dMinL2SquInt = (uint32_t)(dMinL3 * dMinL3 * sFacLevel2 * sFacLevel2);
	uint32_t dMinL1SquInt = (uint32_t)(dMinL3 * dMinL3 * sFacLevel1 * sFacLevel1);

	if (distMtrSquInt <= dMinL3SquInt) {
		newAlarmLevel = ALARM_LEVEL_3;

	} else if (distMtrSquInt <= dMinL2SquInt) {
		newAlarmLevel = ALARM_LEVEL_2;

	} else if (distMtrSquInt <= dMinL1SquInt) {
		newAlarmLevel = ALARM_LEVEL_1;
	}

	if (newAlarmLevel == 0) {
		return NULL;
	}

//	ocapLogStrInt("OCAP-ALARM-LVL", (int)newAlarmLevel);

	TAlarmState *alarmState = alarmStateListAdd(f, newAlarmLevel, t);
	return alarmState;
}
