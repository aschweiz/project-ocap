//
// world_coords.h
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

#ifndef __WORLD_COORD_H__
#define __WORLD_COORD_H__

#include <stdio.h>

// A quadratic area.
class WorldCoords {
public:
    void setLonLatAltRanges(
        double lonMinDeg, double lonMaxDeg, // x
        double latMinDeg, double latMaxDeg, // y
        double altMinMtr); // z

    // Project into -1..1
    double lonDegToWorld(double lonDeg); // x
    double latDegToWorld(double latDeg); // y
    // Same scale as lon/lat but unlimited range
    double altMtrToWorld(double altMtr); // z

public:
    double f = 1;
    // Longitude (x)
    double worldLonMinDeg = 0;
    double worldLonCenterDeg = 0;
    double worldLonMaxDeg = 0;
    // Latitude (y)
    double worldLatMinDeg = 0;
    double worldLatCenterDeg = 0;
    double worldLatMaxDeg = 0;
    // Altitude (z)
    double worldAltMinMtr = 0;
};

#endif // __WORLD_COORD_H__
