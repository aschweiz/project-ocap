//
// OCAP - Open Collision Avoidance Protocol
//
// Logging interface for the OCAP library.
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

#ifndef __OCAP_LOG_H__
#define __OCAP_LOG_H__ 1

#include <inttypes.h>
#include "Vector.h"
#include "FlightObjectOwn.h"
#include "FlightPathExtrapolation.h"

void ocapLogStrInt(const char *str, int i);
void ocapLogStrIntInt(const char *str, int i, int j);

void ocapLogFlVec(int own, TVector *pos, TVector *vel);
void ocapLogFlTN(int nr, TVector *vecT, TVector *vecN);

void ocapLogFlOwn(TFlightObjectOwn *flOwn);
void ocapLogFlOwnPath(TVector *ownFlightPath);
void ocapLogFlOtherPath1(TVector *otherPos, int t);
void ocapLogFlOtherTs(int rxTs, int ts);

void ocapLogFpe(int own, TFlightPathExtrapolationData *fpe);
void ocapLogRZxyV(int own, TFlightPathExtrapolationData *fpe);
void ocapLogModelZxyzV10(int own, EOcapPathModel pathModel, int zx, int zy, int zz, int v10);

#endif // __OCAP_LOG_H__

