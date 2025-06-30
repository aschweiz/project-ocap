//
// igc_parser.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Simple parser for IGC files.
//
// 07.11.2023 ASR  First version
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

#include "igc_parser.h"
#include <fstream>
#include <iostream>
#include <string>

static int igcHhmmssToSeconds(int hhmmss);

double IgcSet::getLatMin()
{
    double latMin = 360;
    for (auto it = files.begin(); it != files.end(); it++) {
        if (it->second->latMin < latMin) {
            latMin = it->second->latMin;
        }
    }
    return latMin;
}

double IgcSet::getLatMax()
{
    double latMax = -360;
    for (auto it = files.begin(); it != files.end(); it++) {
        if (it->second->latMax > latMax) {
            latMax = it->second->latMax;
        }
    }
    return latMax;
}

double IgcSet::getLonMin()
{
    double lonMin = 360;
    for (auto it = files.begin(); it != files.end(); it++) {
        if (it->second->lonMin < lonMin) {
            lonMin = it->second->lonMin;
        }
    }
    return lonMin;
}

double IgcSet::getLonMax()
{
    double lonMax = -360;
    for (auto it = files.begin(); it != files.end(); it++) {
        if (it->second->lonMax > lonMax) {
            lonMax = it->second->lonMax;
        }
    }
    return lonMax;
}

int IgcParser::parseIgcFile(std::string path, IgcFile **file)
{
    // Format specification: https://xp-soaring.github.io/igc_file_format/igc_format_2008.html

    std::ifstream igcFileStream(path);

    *file = nullptr;

    int minTime = 999999;
    int maxTime = 0;

    double minLat = 360;
    double maxLat = -360;
    double minLon = 360;
    double maxLon = -360;

    if (igcFileStream.is_open()) {
        auto igcFile = new IgcFile();
        *file = igcFile;

        std::string line;
        while (getline(igcFileStream, line)) {

            if (line.size() == 0) {
                continue;
            }

            if (line[0] == 'B') {
                if (line.size() < 35) {
                    std::cout << "Error:" << line << "\n";
                } else {
                    // Timestamp 080515
                    auto timePart = line.substr(1, 6);
                    int timeValRaw = std::stoi(timePart);
                    int timeVal = igcHhmmssToSeconds(timeValRaw);
                    // printf("timeVal = %d %d\n",timeValRaw, timeVal);
                    if (timeVal < minTime) {
                        minTime = timeVal;
                    }
                    if (timeVal > maxTime) {
                        maxTime = timeVal;
                    }
                    // Latitude 4656833N
                    auto latDegPart = line.substr(7, 2);
                    int latDeg = std::stoi(latDegPart);
                    auto latMinPart = line.substr(9, 5);
                    int latMin = std::stoi(latMinPart);
                    if (line[14] != 'N' && line[14] != 'S') {
                        std::cout << "Error (invalid latitude):" << line << "\n";
                        continue;
                    }
                    bool isSouth = line[14] == 'S';
                    double latVal = (double)latDeg + ((double)latMin / 1000) / 60;
                    if (isSouth) {
                        latVal = -latVal;
                    }
                    if (latVal < minLat) {
                        minLat = latVal;
                    }
                    if (latVal > maxLat) {
                        maxLat = latVal;
                    }
                    // Longitude 00727188E
                    auto lonDegPart = line.substr(15, 3);
                    int lonDeg = std::stoi(lonDegPart);
                    auto lonMinPart = line.substr(18, 5);
                    int lonMin = std::stoi(lonMinPart);
                    if (line[23] != 'W' && line[23] != 'E') {
                        std::cout << "Error (invalid longitude):" << line << "\n";
                        continue;
                    }
                    bool isWest = line[23] == 'W';
                    double lonVal = (double)lonDeg + ((double)lonMin / 1000) / 60;
                    if (isWest) {
                        lonVal = -lonVal;
                    }
                    if (lonVal < minLon) {
                        minLon = lonVal;
                    }
                    if (lonVal > maxLon) {
                        maxLon = lonVal;
                    }
                    // Altitude
                    if (line[24] != 'A') {
                        std::cout << "Error (invalid altituude):" << line << "\n";
                        continue;
                    }
                    auto altBaroPart = line.substr(25, 5);
                    int altBaro = std::stoi(altBaroPart);
                    auto altGpsPart = line.substr(30, 5);
                    int altGps = std::stoi(altGpsPart);

                    auto igcPosition = new IgcPosition();
                    igcPosition->timestamp = timeVal;
                    igcPosition->lat = latVal;
                    igcPosition->lon = lonVal;
                    igcPosition->altGps = altGps;
                    igcPosition->altBaro = altBaro;

                    igcFile->positions[timeVal] = igcPosition;
                }
            }
        }

        igcFile->timestampMin = minTime;
        igcFile->timestampMax = maxTime;
        printf("igcFile time range [%d,%d]\n", minTime, maxTime);
        igcFile->latMin = minLat;
        igcFile->latMax = maxLat;
        igcFile->lonMin = minLon;
        igcFile->lonMax = maxLon;

        igcFileStream.close();
    }

    return 0;
}

static int igcHhmmssToSeconds(int hhmmss)
{
    int sec = hhmmss % 100;
    hhmmss /= 100;
    int min = hhmmss % 100;
    hhmmss /= 100;
    int result = 3600 * hhmmss + 60 * min + sec;
    return result;
}
