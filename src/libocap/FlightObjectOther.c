//
// OCAP - Open Collision Avoidance Protocol
//
// Information on a nearby flight object (paraglider, helicopter,
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

#include "FlightObjectOther.h"


void flightObjectOtherInit(TFlightObjectOther *f, uint32_t id)
{
	f->id = id;
	f->state = FOS_INIT;
	f->ts = 0;
	f->timeSinceLastRxSec = 0;
	f->rxTs = 0;
	vectorClear(&f->pos_i0);
	vectorClear(&f->vel_i0);
	vectorClear(&f->rxPos);
	vectorClear(&f->rxVel);
	vectorClear(&f->z);
}

void flightObjectOtherRelease(TFlightObjectOther *f)
{
	flightObjectOtherInit(f, 0);
	f->state = FOS_UNALLOCATED;
}

void flightObjectOtherCopy(TFlightObjectOther *target, TFlightObjectOther *source)
{
	target->id = source->id; // 0x004E7593
	target->state = source->state;

	target->rxTs = source->rxTs;
	vectorCopy(&target->rxPos, &source->rxPos);
	vectorCopy(&target->rxVel, &source->rxVel);

	target->ts = source->ts;
	target->timeSinceLastRxSec = source->timeSinceLastRxSec;
	vectorCopy(&target->pos_i0, &source->pos_i0);
	vectorCopy(&target->vel_i0, &source->vel_i0);

	vectorCopy(&target->z, &source->z);
}

void flightObjectOtherActivateRxData(TFlightObjectOther *f)
{
	f->ts = f->rxTs;
	vectorCopy(&f->pos_i0, &f->rxPos);
	vectorCopy(&f->vel_i0, &f->rxVel);
}

