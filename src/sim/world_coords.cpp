//
// world_coords.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Boundaries of the simulated world.
//
// 15.11.2023 ASR  First version
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

#include "world_coords.h"
#include <cmath>

void WorldCoords::setLonLatAltRanges(
    double lonMinDeg, double lonMaxDeg, // x
    double latMinDeg, double latMaxDeg, // y
    double altMinMtr) // z
{
    double lonDelta = lonMaxDeg - lonMinDeg;
    worldLonCenterDeg = lonMinDeg + 0.5 * lonDelta;
    double latDelta = latMaxDeg - latMinDeg;
    worldLatCenterDeg = latMinDeg + 0.5 * latDelta;
    worldAltMinMtr = altMinMtr;

    // Project Deg to -1..1
    double cosLat = cos(worldLatCenterDeg / 180.0 * M_PI);
    double lonDeltaCorr = lonDelta / cosLat;
    f = lonDeltaCorr > latDelta ? (2 / lonDeltaCorr) : (2 / latDelta);
    printf("World center: lon=%.6lf, lat=%.6lf\n", worldLonCenterDeg, worldLatCenterDeg);
}

double WorldCoords::lonDegToWorld(double lonDeg)
{
    double cosLat = cos(worldLatCenterDeg / 180.0 * M_PI);
    double lonDelta = lonDeg - worldLonCenterDeg;
    return lonDelta * f * cosLat * -1; // -1 due to coordinate system orientation
}

double WorldCoords::latDegToWorld(double latDeg)
{
    double latDelta = latDeg - worldLatCenterDeg;
    return latDelta * f;
}

double WorldCoords::altMtrToWorld(double altMtr)
{
    // 1 deg = 111'111.1 mtr, go from mtr to deg, then to world -1..1.
    return (altMtr - worldAltMinMtr) / 111111.1 * f;
}
