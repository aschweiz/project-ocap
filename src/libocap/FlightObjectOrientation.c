//
// FlightObjectOrientation.c
//
// OCAP - Open Collision Avoidance Protocol
//
// Getting the orientation of one airplane to another.
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

#include <math.h>
#include "FlightObjectOrientation.h"


int flightObjectOrientationCalculate(
  TFlightObjectOrientation *result,
  TVector *ownToOther,
  TVector *ownSpeed)
{
  if (!ownToOther || !ownSpeed) {
    return 0;
  }

  // From Origin to Other.
  TVector *d = ownToOther;

  // Distance to the other aircraft.
  float distMtr = vectorGetLength(d);
  result->distanceMeters = (int)distMtr;

  // Angle to the other aircraft.
  float alpha = atan2(d->y, d->x);

  // Angle of our velocity vector.
  float beta = atan2(ownSpeed->y, ownSpeed->x);

  // Left = -90, ahead = 0, right = +90.
  float deltaAngle = beta - alpha;
  int deltaAngleDeg = (int)(deltaAngle * 180 / 3.14159265);

  if (deltaAngleDeg > 180) {
    deltaAngleDeg = 360 - deltaAngleDeg;
  } else if (deltaAngleDeg < -180) {
    deltaAngleDeg = 360 + deltaAngleDeg;
  }

  result->directionDeg = deltaAngleDeg;
  return 1;
}
