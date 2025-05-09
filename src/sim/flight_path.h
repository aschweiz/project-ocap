//
// flight_path.h
//
// OCAP - Open Collision Avoidance Protocol
//
// Single flight path of an aircraft.
//
// 09.04.2024 ASR  First version
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

#ifndef __FLIGHT_PATH_H__
#define __FLIGHT_PATH_H__

#include "linalg.h"

using namespace linalg;

typedef enum {
    FLIGHT_PATH_FILE_TYPE_FLP,
    FLIGHT_PATH_FILE_TYPE_PFLAG,
} EFlightPathFileType;

// A quadratic area.
class FlightPath {
public:
    FlightPath();
    virtual ~FlightPath();

    bool LoadFile(const char *flpPath, EFlightPathFileType fileType);

    double GetLonMinDeg();
    double GetLonMaxDeg();
    double GetLatMinDeg();
    double GetLatMaxDeg();
    double GetAltMinMtr();
    double GetAltMaxMtr();

    int GetTimestampSecMin();
    int GetTimestampSecMax();

    // X=LON (deg), Y=LAT (deg), Z=ALT (mtr)
    bool GetPositionDegDegMtrAtMillisec(long ms, Vector3d *vec);

    // X=LON, Y=LAT, Z=ALT, all in 1m/s units.
    bool GetVelocityAtMillisec(long ms, Vector3d *vec);

private:
    // 3D bounding box
    double lonMinDeg;
    double lonMaxDeg;
    double latMinDeg;
    double latMaxDeg;
    double altMinMtr;
    double altMaxMtr;

    // x=lonDeg, y=latDeg, z=altMtr
    Vector3d **positionVectors = nullptr;
    Vector3d **velocityVectors = nullptr; // may stay null
    int nofDatapoints = 0;
};

#endif // __FLIGHT_PATH_H__
