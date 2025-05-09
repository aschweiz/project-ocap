//
// FlightObjectList.c
//
// OCAP - Open Collision Avoidance Protocol
//
// List of FlightObject instances that we are currently tracking.
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

#include "Configuration.h"
#include "FlightObjectList.h"

static TFlightObjectOwn sFlightObjectOwn;

static TFlightObjectOther sFlightObjects[FLIGHT_OBJECT_LIST_LENGTH];
static int sCount;


void flightObjectListInit(uint32_t idOwn)
{
	flightObjectOwnInit(&sFlightObjectOwn, idOwn);

	sCount = 0;
	// Start with all flight objects 'unallocated'.
	for (int i = 0; i < FLIGHT_OBJECT_LIST_LENGTH; i++) {
		TFlightObjectOther *f = &sFlightObjects[i];
		f->state = FOS_UNALLOCATED;
	}
}

TFlightObjectOwn *flightObjectListGetOwn(void)
{
	return &sFlightObjectOwn;
}

int flightObjectListGetOtherCount(void)
{
	return sCount;
}

TFlightObjectOther *flightObjectListGetOtherAtIndex(int i)
{
	return &sFlightObjects[i];
}

TFlightObjectOther *flightObjectListGetOther(uint32_t id)
{
	for (int i = 0; i < sCount; i++) {
		TFlightObjectOther *f = &sFlightObjects[i];
		if (f->id == id) {
			return f;
		}
	}
	return 0L;
}

TFlightObjectOther *flightObjectListAddOther(uint32_t id)
{
	// Return if the list is full.
	if (sCount >= FLIGHT_OBJECT_LIST_LENGTH) {
		return 0;
	}

	// Add the entry at the end.
	int ix = sCount++;
	TFlightObjectOther *f = &sFlightObjects[ix];
	flightObjectOtherInit(f, id);
	return f;
}

void flightObjectListRemoveOtherAtIndex(int ix)
{
	// Release the flight object.
	TFlightObjectOther *f = &sFlightObjects[ix];
	if (f->state == FOS_UNALLOCATED) {
		return;
	}

	// Fill the hole if we remove an inner entry.
	int lastIx = sCount - 1;
	if (ix < lastIx) {
		// It's an inner entry; move the last entry to the inner.
		TFlightObjectOther *lastF = &sFlightObjects[lastIx];
		flightObjectOtherCopy(f, lastF);
		flightObjectOtherRelease(lastF);
	} else {
		// It's the last entry, simply release it.
		flightObjectOtherRelease(f);
	}

	sCount--;
	return;
}

