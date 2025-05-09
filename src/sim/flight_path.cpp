//
// flight_path.cpp
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

#include "flight_path.h"
#include "linalg.h"
#include "lon_lat_util.h"
#include "parse_pflag.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using namespace linalg;

FlightPath::FlightPath()
{
}

FlightPath::~FlightPath()
{
    for (int i = 0; i < nofDatapoints; i++) {
        delete positionVectors[i];
        delete velocityVectors[i];
    }
    delete[] positionVectors;
    delete[] velocityVectors;
}

bool FlightPath::LoadFile(const char *filePath, EFlightPathFileType fileType)
{
    lonMinDeg = 181;
    lonMaxDeg = -181;
    latMinDeg = 91;
    latMaxDeg = -91;
    altMinMtr = 9999999;
    altMaxMtr = -9999999;

    std::ifstream flpFileStream(filePath);
    if (!flpFileStream.is_open()) {
        std::cerr << "Failed to parse file: " << filePath << std::endl;
        return false;
    }

    std::vector<linalg::Vector3d*> tmpPositionVectors;
    std::vector<linalg::Vector3d*> tmpVelocityVectors;

    std::string line;
    while (getline(flpFileStream, line)) {
        if (line.size() == 0) {
            continue;
        }

        linalg::Vector3d *p = nullptr;
        linalg::Vector3d *v = nullptr;

        if (fileType == FLIGHT_PATH_FILE_TYPE_FLP) {
            double lonDeg = 0, latDeg = 0, altMtr = 0;
            if (3 == sscanf(line.c_str(), "%lf;%lf;%lf", &lonDeg, &latDeg, &altMtr)) {
                p = new Vector3d(lonDeg, latDeg, altMtr);
                tmpPositionVectors.push_back(p);
            }
        } else if (fileType == FLIGHT_PATH_FILE_TYPE_PFLAG) {
            TVector pos, vel;
            int pflagValid = pflaDecodeG_A(line.c_str(), &pos, &vel);
            if (pflagValid) {
                p = new Vector3d(pos.x, pos.y, pos.z);
                v = new Vector3d(vel.x, vel.y, vel.z);
                tmpPositionVectors.push_back(p);
                tmpVelocityVectors.push_back(v);
            }
        }

        // Extend the world for each valid position vector.
        if (p) {
            lonMinDeg = std::min(lonMinDeg, p->X());
            lonMaxDeg = std::max(lonMaxDeg, p->X());
            latMinDeg = std::min(latMinDeg, p->Y());
            latMaxDeg = std::max(latMaxDeg, p->Y());
            altMinMtr = std::min(altMinMtr, p->Z());
            altMaxMtr = std::max(altMaxMtr, p->Z());
        }
    }

    int hasVelocityVectors = tmpVelocityVectors.size() == tmpPositionVectors.size();

    velocityVectors = nullptr;
    positionVectors = new linalg::Vector3d*[tmpPositionVectors.size()];
    if (hasVelocityVectors) {
        velocityVectors = new linalg::Vector3d*[tmpPositionVectors.size()];
    }

    for (int i = 0; i < tmpPositionVectors.size(); i++) {
        positionVectors[i] = tmpPositionVectors.at(i);
        if (hasVelocityVectors) {
            velocityVectors[i] = tmpVelocityVectors.at(i);
        }
    }

    nofDatapoints = tmpPositionVectors.size();
    std::cout << "Imported flight path of length " << nofDatapoints << std::endl;
    std::cout << "Bounding box: lon " << lonMinDeg << ".." << lonMaxDeg << std::endl;
    std::cout << "              lat " << latMinDeg << ".." << latMaxDeg << std::endl;
    std::cout << "              alt " << altMinMtr << ".." << altMaxMtr << std::endl;

    flpFileStream.close();
    return true;
}

double FlightPath::GetLonMinDeg()
{
    return lonMinDeg;
}

double FlightPath::GetLonMaxDeg()
{
    return lonMaxDeg;
}

double FlightPath::GetLatMinDeg()
{
    return latMinDeg;
}

double FlightPath::GetLatMaxDeg()
{
    return latMaxDeg;
}

double FlightPath::GetAltMinMtr()
{
    return altMinMtr;
}

double FlightPath::GetAltMaxMtr()
{
    return altMaxMtr;
}

int FlightPath::GetTimestampSecMin()
{
    return 0;
}

int FlightPath::GetTimestampSecMax()
{
    return nofDatapoints - 1;
}

bool FlightPath::GetPositionDegDegMtrAtMillisec(long ms, Vector3d *vec)
{
    if (!positionVectors) {
        std::cerr << "Flight path contains no data points" << std::endl;
        return false;
    }

    if (ms <= 0) {
        *vec = *positionVectors[0];
        return true;
    }

    int ix = ms / 1000;
    if (ix >= nofDatapoints) {
        *vec = *positionVectors[nofDatapoints - 1];
        return true;
    }

    // TODO: linear interpolation
    *vec = *positionVectors[ix];
    return true;
}

// X=LON, Y=LAT, Z=ALT, all in 1m/s units.
bool FlightPath::GetVelocityAtMillisec(long ms, Vector3d *vec)
{
    if (!velocityVectors) {
        Vector3d vOldDeg(0, 0, 0);
        Vector3d vNewDeg(0, 0, 0);
        int measSec = 1;
        if (!GetPositionDegDegMtrAtMillisec(ms, &vNewDeg)
            || !GetPositionDegDegMtrAtMillisec(ms - 1000 * measSec, &vOldDeg)) {
            return false;
        }

        Vector3d vOldMtr(0, 0, 0);
        Vector3d vNewMtr(0, 0, 0);
        lonDegLatDegAltMtrToMtrMtrMtr(vOldDeg.GetVector(), vOldMtr.GetVector());
        lonDegLatDegAltMtrToMtrMtrMtr(vNewDeg.GetVector(), vNewMtr.GetVector());

        double dx = (vNewMtr.X() - vOldMtr.X()) / measSec;
        double dy = (vNewMtr.Y() - vOldMtr.Y()) / measSec;
        double dz = (vNewMtr.Z() - vOldMtr.Z()) / measSec;
        vec->Set(dx, dy, dz);

        return true;
    }

    if (ms <= 0) {
        *vec = *velocityVectors[0];
        return true;
    }

    int ix = ms / 1000;
    if (ix >= nofDatapoints) {
        *vec = *velocityVectors[nofDatapoints - 1];
        return true;
    }

    // TODO: linear interpolation
    *vec = *velocityVectors[ix];
    return true;
}
