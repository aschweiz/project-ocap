//
// FlightObjectList.h
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

#ifndef __FLIGHT_OBJECT_LIST_H__
#define __FLIGHT_OBJECT_LIST_H__ 1

#include <inttypes.h>
#include "FlightObjectOwn.h"
#include "FlightObjectOther.h"

void flightObjectListInit(uint32_t idOwn);

TFlightObjectOwn *flightObjectListGetOwn(void);

int flightObjectListGetOtherCount(void);
TFlightObjectOther *flightObjectListGetOtherAtIndex(int i);

TFlightObjectOther *flightObjectListGetOther(uint32_t id);

// Returns the index of the new flight object, or -1, if there was no space.
TFlightObjectOther *flightObjectListAddOther(uint32_t id);

void flightObjectListRemoveOtherAtIndex(int ix);

#endif // __FLIGHT_OBJECT_LIST_H__
