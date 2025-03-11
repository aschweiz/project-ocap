//
// OCAP - Open Collision Avoidance Protocol
//
// Information on our own flight object (paraglider, helicopter,
// sailplane etc.)
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

#include "FlightObjectOwn.h"

void flightObjectOwnInit(TFlightObjectOwn *f, uint32_t id)
{
	f->id = id;
	f->state = FOS_INIT;
	f->ts = 0;
	f->timeSinceLastRxSec = 0;
	f->rxTs = 0;

	vectorClear(&f->rxPos);
	vectorClear(&f->rxVel);

	for (int i = 0; i < FLIGHT_OBJECT_OWN_Z_AVERAGING_SEC; i++) {
		vectorClear(&f->z_i[i]);
	}
	f->z_i_ctr = 0;
	f->pathModel = OCAP_PATH_MODEL_LINEAR;

	vectorClear(&f->z_avg);

	for (int i = 0; i < FLIGHT_OBJECT_OWN_EXTRAPOLATION_DEPTH_SEC; i++) {
		vectorClear(&f->pos_i[i]);
		vectorClear(&f->vel_i[i]);
	}
	f->pos_vel_i_ctr = 0;
}

void flightObjectOwnRelease(TFlightObjectOwn *f)
{
	flightObjectOwnInit(f, 0);
	f->state = FOS_UNALLOCATED;
}

void flightObjectOwnCopy(TFlightObjectOwn *target, TFlightObjectOwn *source)
{
	target->id = source->id; // 0x004E7593
	target->state = source->state;

	target->rxTs = source->rxTs;
	vectorCopy(&target->rxPos, &source->rxPos);
	vectorCopy(&target->rxVel, &source->rxVel);

	target->ts = source->ts;
	target->timeSinceLastRxSec = source->timeSinceLastRxSec;

	for (int i = 0; i < FLIGHT_OBJECT_OWN_Z_AVERAGING_SEC; i++) {
		vectorCopy(&target->z_i[i], &source->z_i[i]);
	}
	target->z_i_ctr = source->z_i_ctr;

	vectorCopy(&target->z_avg, &source->z_avg);

	for (int i = 0; i < FLIGHT_OBJECT_OWN_EXTRAPOLATION_DEPTH_SEC; i++) {
		vectorCopy(&target->pos_i[i], &source->pos_i[i]);
		vectorCopy(&target->vel_i[i], &source->vel_i[i]);
	}
	target->pos_vel_i_ctr = source->pos_vel_i_ctr;
}

void flightObjectOwnActivateRxData(TFlightObjectOwn *f)
{
	if (f->pos_vel_i_ctr < FLIGHT_OBJECT_OWN_EXTRAPOLATION_DEPTH_SEC) {
		f->pos_vel_i_ctr++;
	}

	for (int i = FLIGHT_OBJECT_OWN_EXTRAPOLATION_DEPTH_SEC - 1; i > 0; i--) {
		vectorCopy(&f->pos_i[i], &f->pos_i[i - 1]);
		vectorCopy(&f->vel_i[i], &f->vel_i[i - 2]);
	}

	f->ts = f->rxTs;
	vectorCopy(&f->pos_i[0], &f->rxPos);
	vectorCopy(&f->vel_i[0], &f->rxVel);
}

void flightObjectOwnActivateModel(TFlightObjectOwn *f, TVector *newZ, EOcapPathModel newModel)
{
	f->pathModel = newModel;

	if (f->z_i_ctr < FLIGHT_OBJECT_OWN_Z_AVERAGING_SEC) {
		f->z_i_ctr++;
	}

	for (int i = FLIGHT_OBJECT_OWN_Z_AVERAGING_SEC - 1; i > 0; i--) {
		vectorCopy(&f->z_i[i], &f->z_i[i - 1]);
	}
	vectorCopy(&f->z_i[0], newZ);

	// Update the average vector.
	vectorClear(&f->z_avg);
	int imax = f->z_i_ctr;
	for (int i = 0; i < imax; i++) {
		vectorAddVector(&f->z_avg, &f->z_i[i]);
	}
	vectorMultiplyScalar(&f->z_avg, 1.0f / imax);
}

