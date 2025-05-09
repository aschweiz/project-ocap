//
// FlightPathExtrapolation.c
//
// OCAP - Open Collision Avoidance Protocol
//
// Predicting the future flight path of an object by considering the
// object's velocity and radial component of acceleration.
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

#include "FlightPathExtrapolation.h"
#include "OcapLog.h"

#define ABS_D(X) ((X < 0) ? (-X) : (X))


static void flightPathExtrapolationFinalizePreparation(
		TFlightPathExtrapolationData *fpe);


// Initialization step with 2x rv.
// Fills z_vec, v, r, isCurve.
void flightPathExtrapolationPrepare2rv(
	TFlightPathExtrapolationData *fpe)
{
	// The _length_ of the velocity vector is assumed to be constant along the curve.

	float vNew = vectorGetLength(&fpe->vi_vec);
	float vOld = vectorGetLength(&fpe->v0_vec);
	fpe->v = vNew;

	// Early abort if the object is static.
	if (vNew < 0.5 && vOld < 0.5) {
		fpe->predictionModel = OCAP_PATH_MODEL_SPHERIC;
		vectorCopy(&fpe->z_vec, &fpe->ri_vec);
		fpe->r = 0;
		return;
	}

	// Limit the vector length to avoid NANs in the subsequent calculations.
	if (vNew < 0.001) {
		vNew = 0.001;
	}
	if (vOld < 0.001) {
		vOld = 0.001;
	}

	// T_old, T_New

	TVector old_T_vec; // tangential in r0
	vectorCopy(&old_T_vec, &fpe->v0_vec);
	vectorMultiplyScalar(&old_T_vec, 1.0 / vOld);

	TVector new_T_vec; // tangential in ri
	vectorCopy(&new_T_vec, &fpe->vi_vec);
	vectorMultiplyScalar(&new_T_vec, 1.0 / vNew);

	// N_old

	TVector tmp_vec;
	vectorCopy(&tmp_vec, &old_T_vec);
	float tT0 = vectorMultiplyDot(&new_T_vec, &tmp_vec);
	vectorMultiplyScalar(&tmp_vec, tT0);

	TVector old_N_vec; // radial
	vectorCopy(&old_N_vec, &new_T_vec);

	vectorSubtractVector(&old_N_vec, &tmp_vec);
	float nLen = vectorGetLength(&old_N_vec);
	// TODO HIGH find a better solution to avoid NAN here!
	if (nLen < .001f) {
		nLen = .001f;
	}
	vectorMultiplyScalar(&old_N_vec, 1.0 / nLen);

	ocapLogFlTN(0, &old_T_vec, &old_N_vec);

	// N_new

	vectorCopy(&tmp_vec, &new_T_vec);
	float tT1 = vectorMultiplyDot(&old_T_vec, &tmp_vec);
	vectorMultiplyScalar(&tmp_vec, tT1);

	TVector new_N_vec; // radial
	vectorCopy(&new_N_vec, &tmp_vec);
	vectorSubtractVector(&new_N_vec, &old_T_vec);

	float nLen2 = vectorGetLength(&new_N_vec);
	// TODO HIGH find a better solution to avoid NAN here!
	if (nLen2 < .001f) {
		nLen2 = .001f;
	}
	vectorMultiplyScalar(&new_N_vec, 1.0 / nLen2);

	// TODO REMOVE (for testing only)
	vectorCopy(&fpe->ni_vec, &new_N_vec);

	ocapLogFlTN(1, &new_T_vec, &new_N_vec);

	// r, Z

	// If the N vectors are nearly equal, we can use a linear model.
	// At least 1 axis should differ by 1 in 1000.
	if ((int)ABS_D((old_N_vec.x - new_N_vec.x) * 1000) == 0
			&& (int)ABS_D((old_N_vec.y - new_N_vec.y) * 1000) == 0
			&& (int)ABS_D((old_N_vec.z - new_N_vec.z) * 1000) == 0) {

		fpe->predictionModel = OCAP_PATH_MODEL_LINEAR;
		return;
	}

	// r := (r1.x - r0.x) / (N0.x - N1.x)  or y, z if denom is small
	float r;
	float dx = old_N_vec.x - new_N_vec.x;
	float dy = old_N_vec.y - new_N_vec.y;
	float dz = old_N_vec.z - new_N_vec.z;
	if (ABS_D(dz) > ABS_D(dx) && ABS_D(dz) > ABS_D(dy)) {
		r = (fpe->ri_vec.z - fpe->r0_vec.z) / dz;
	} else if (ABS_D(dy) > ABS_D(dx)) {
		r = (fpe->ri_vec.y - fpe->r0_vec.y) / dy;
	} else {
		r = (fpe->ri_vec.x - fpe->r0_vec.x) / dx;
	}
	fpe->r = r;

	// _Z_ := _ri_ + r * _Ni_
	vectorCopy(&fpe->z_vec, &new_N_vec);
	vectorMultiplyScalar(&fpe->z_vec, r);
	vectorCopy(&fpe->r_z_vec, &fpe->z_vec);
	vectorAddVector(&fpe->z_vec, &fpe->ri_vec);

	// Default model (within lower and upper bound) is the arc model.
	fpe->predictionModel = OCAP_PATH_MODEL_ARC;
	float lowerBound = vNew * 0.5f;
	float upperBound = vNew * 2024.0f;

	// Empirically found during testing: Add an absolute limit for the
	// lower bound to avoid arc model for small radii (below, say, 5 meters).
	if (lowerBound < 5.0f) {
		lowerBound = 5.0f;
	}

	// Lower bound is 0.5*v, below that, we use a spheric approximation.
	if (r < lowerBound) {
		fpe->predictionModel = OCAP_PATH_MODEL_SPHERIC;
		return;
	}

	// Upper bound is 2024*v, above that, we use a linear approximation.
	if (r >= upperBound) {
		fpe->predictionModel = OCAP_PATH_MODEL_LINEAR;
	}

	// printf("    r  = %8.4f\n", r);
	// printf("    Z  = (%8.4f,%8.4f,%8.4f)\n", Z_vec->x, Z_vec->y, Z_vec->z);

	flightPathExtrapolationFinalizePreparation(fpe);
}

// Initialization step with 1x rvz.
// Set r0/v0 and ri/vi to the same data.
void flightPathExtrapolationPrepareRvz(
	TFlightPathExtrapolationData *fpe)
{
	// The _length_ of the velocity vector is assumed to be constant along the curve.
	float v = vectorGetLength(&fpe->vi_vec);
	fpe->v = v;

	TVector tmpVec;
	vectorCopy(&tmpVec, &fpe->ri_vec);
	vectorSubtractVector(&tmpVec, &fpe->z_vec);
	fpe->r = vectorGetLength(&tmpVec);

	flightPathExtrapolationFinalizePreparation(fpe);
}

void flightPathExtrapolationExecute(
	TFlightPathExtrapolationData *fpe)
{
	if (fpe->predictionModel == OCAP_PATH_MODEL_LINEAR) {
		// 1 second flight on a straight line.
		vectorAddVector(&fpe->ri_vec, &fpe->vi_vec);
		return;
	}

	if (fpe->predictionModel == OCAP_PATH_MODEL_SPHERIC) {
		// Assume the aircraft is static in point z.
		vectorCopy(&fpe->ri_vec, &fpe->z_vec);
		return;
	}

	// New position: _r(new)_ := t/r * (_Z_ - _r(old)_) + _r(old)_ + A * _T(old)_

	// Point Q(old) on the line from r(old) to Z
	TVector old_Q_vec; // _Q(old)_ := t/r * (_Z_ - _r(old)) ...
	vectorCopy(&old_Q_vec, &fpe->z_vec);
	vectorSubtractVector(&old_Q_vec, &fpe->ri_vec);
	vectorMultiplyScalar(&old_Q_vec, fpe->t_div_r);
	vectorAddVector(&old_Q_vec, &fpe->ri_vec); // ... + _r(old)_

	// printf("    Q_old  = (%8.4f,%8.4f,%8.4f)\n", old_Q_vec.x, old_Q_vec.y, old_Q_vec.z);

	// Point r(new) is theta*_v(old)_ away from Q(old).
	TVector theta_v_old_vec;
	vectorCopy(&theta_v_old_vec, &fpe->vi_vec);
	vectorMultiplyScalar(&theta_v_old_vec, fpe->theta);

	// Go from _r(old)_ to _r(new)_.
	TVector tmp_new_r_vec;
	vectorCopy(&tmp_new_r_vec, &old_Q_vec);
	vectorAddVector(&tmp_new_r_vec, &theta_v_old_vec);

	// New velocity: _v(new)_ := 1/theta * (_Qi_ - _r(old))
	//            = 1/theta * (_r(new)_ + t/r * (_Z_ - _r(new)_) - _r(old))

	// Point Q(new) on the line from r(new) to Z
	TVector new_Q_vec; // _Q(new) := t/r * (_Z_ - r(new)) ...
	vectorCopy(&new_Q_vec, &fpe->z_vec);
	vectorSubtractVector(&new_Q_vec, &tmp_new_r_vec);
	vectorMultiplyScalar(&new_Q_vec, fpe->t_div_r);
	vectorAddVector(&new_Q_vec, &tmp_new_r_vec); // ... + _r(new)_

	// printf("    Q_new  = (%8.4f,%8.4f,%8.4f)\n", new_Q_vec.x, new_Q_vec.y, new_Q_vec.z);

	// Vector v(new) is parallel to (_Q(new)_ - _r(old)_).
	TVector tmp_new_v_vec;
	vectorCopy(&tmp_new_v_vec, &new_Q_vec);
	vectorSubtractVector(&tmp_new_v_vec, &fpe->ri_vec); // ... - _r(old)_

	// *theta converts from v to d_r_, here we go back from d_r_ to v:
	vectorMultiplyScalar(&tmp_new_v_vec, 1.0 / fpe->theta);

	vectorCopy(&fpe->ri_vec, &tmp_new_r_vec);
	vectorCopy(&fpe->vi_vec, &tmp_new_v_vec);
}

// Finalize the preparation by calculating theta and t_div_r.
static void flightPathExtrapolationFinalizePreparation(
		TFlightPathExtrapolationData *fpe)
{
	float v = fpe->v;
	float r = fpe->r;

	// alpha, theta
	float alpha = v / r;
	float a = r * sinf(alpha); // TODO: Taylor expansion
	fpe->theta = a / v;
	fpe->t_div_r = 1.0 - cosf(alpha); // TODO: Taylor expansion

	// printf("    a        = %8.4f\n", a);
	// printf("    theta    = %8.4f\n", fpe->theta);
	// printf("    t_div_r  = %8.4f\n", fpe->t_div_r);
}
