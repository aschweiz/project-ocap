//
// test_set.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// A test set groups a number of flight pathes from different aircraft for
// a specified amount of time.
//
// 16.04.2024 ASR  First version
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

#include "test_set.h"
#include "flight_path_config.h"
#include <fstream>
#include <iostream>
#include <string.h>

static TestSet *sLoadedTestSet;

inline bool ends_with(std::string const &value, std::string const &ending)
{
    if (ending.size() > value.size()) {
        return false;
    }
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

TestSet *TestSet::LoadedTestSet()
{
    return sLoadedTestSet;
}

TestSet::TestSet()
{
}

TestSet::~TestSet()
{
    if (sLoadedTestSet == this) {
        sLoadedTestSet = nullptr;
    }
    for (int i = 0; i < flightPaths.size(); i++) {
        delete flightPaths[i];
    }
}

bool TestSet::LoadTestSet(const char *testSetPath)
{
    std::ifstream testSetStream(testSetPath);
    if (!testSetStream.is_open()) {
        std::cerr << "Failed to parse test set file: " << testSetPath << std::endl;
        return false;
    }

    std::string flightPathDirectory;

    int fileVersion;
    int ix = 0;
    std::string line;
    while (getline(testSetStream, line)) {
        if (line.size() == 0 || line[0] == '#') {
            continue;
        }

        ix++;

        // First line: version
        if (ix == 1) {
            if (1 != sscanf(line.c_str(), "%d", &fileVersion)) {
                std::cerr << "Failed to parse file version in test set." << std::endl;
                return false;
            }
            if (fileVersion < 1 || fileVersion > 2) {
                std::cerr << "File version not supported: " << fileVersion << std::endl;
                return false;
            }
            continue;
        }

        // Second line: test set name
        if (ix == 2) {
            testSetName = line;
            continue;
        }

        // Third line: flight path directory
        if (ix == 3) {
            flightPathDirectory = line;
            continue;
        }

        // Time span
        if (ix == 4) {
            if (2 != sscanf(line.c_str(), "%lf;%lf", &startTimeSec, &endTimeSec)) {
                std::cerr << "Failed to parse start and end time in test set." << std::endl;
                return false;
            }
            continue;
        }

        // Version 2 and higher includes viewer configuration
        if (fileVersion >= 2) {
            if (ix == 5) {
                if (1 != sscanf(line.c_str(), "rotXZ=%lf", &rotXZ)) {
                    std::cerr << "Failed to parse rotXZ" << std::endl;
                    return false;
                }
                continue;
            }
            if (ix == 6) {
                if (1 != sscanf(line.c_str(), "rotXY=%lf", &rotXY)) {
                    std::cerr << "Failed to parse rotXY" << std::endl;
                    return false;
                }
                continue;
            }
            if (ix == 7) {
                if (1 != sscanf(line.c_str(), "trH=%lf", &trH)) {
                    std::cerr << "Failed to parse trH" << std::endl;
                    return false;
                }
                continue;
            }
            if (ix == 8) {
                if (1 != sscanf(line.c_str(), "trV=%lf", &trV)) {
                    std::cerr << "Failed to parse trV" << std::endl;
                    return false;
                }
                continue;
            }
            if (ix == 9) {
                if (1 != sscanf(line.c_str(), "camR=%lf", &camR)) {
                    std::cerr << "Failed to parse camR" << std::endl;
                    return false;
                }
                continue;
            }
            if (ix == 10) {
                if (2 != sscanf(line.c_str(), "%lf;%lf", &startTimeAutoRunSec, &endTimeAutoRunSec)
                    || startTimeAutoRunSec < 0
                    || endTimeAutoRunSec < startTimeAutoRunSec) {
                    std::cerr << "Failed to parse start and end time for autorun" << std::endl;
                    return false;
                }
                continue;
            }
        }

        // Data lines
        // name; t; x/lon; y/lat; z/alt; file
        const char *cp = line.c_str();
        std::string objName;
        while (*cp != ';') {
            objName += *cp;
            cp++;
        }
        while (*cp == ';' || *cp == ' ' || *cp == '\t') {
            cp++;
        }
        double objT = 0, objDLonDeg = 0, objDLatDeg = 0, objDAltMtr = 0;
        if (4 != sscanf(cp, "%lf;%lf;%lf;%lf", &objT, &objDLonDeg, &objDLatDeg, &objDAltMtr)) {
            // Need to skip entry.
            std::cerr << "Invalid object in test set: " << line << std::endl;
            continue;
        }

        // Look for last semicolon.
        const char *lastCp = cp;
        while (*cp) {
            if (*cp == ';') {
                lastCp = cp;
            }
            cp++;
        }
        if (*lastCp == ';') {
            while (*lastCp == ';' || *lastCp == ' ' || *lastCp == '\t') {
                lastCp++;
            }
            if (*lastCp) {
                std::string fileName = lastCp;

                // printf("obj: '%s', dt: %f, pos: %f,%f,%f, file: '%s'\n",
                //	objName.c_str(), objT, objX, objY, objZ, fileName.c_str());

                std::string filePath = flightPathDirectory;
                filePath += '/';
                filePath += fileName;

                FlightPathConfig *fpc = new FlightPathConfig(objName);
                bool fpcLoaded = false;
                if (ends_with(filePath, ".flp")) {
                    fpcLoaded = fpc->LoadFlp(filePath.c_str());
                } else if (ends_with(filePath, ".pflag")) {
                    fpcLoaded = fpc->LoadPflag(filePath.c_str());
                }
                if (!fpcLoaded) {
                    std::cerr << "Failed to load object flight path in test set: " << line << std::endl;
                    continue;
                }

                fpc->SetTimeOffset(objT);
                fpc->SetPositionOffset(objDLonDeg, objDLatDeg, objDAltMtr);

                flightPaths.push_back(fpc);

                // TODO move to another place!
                // Check for corresponding PKT file.
                std::string pktPath = flightPathDirectory;
                pktPath += '/';
                char filenameBuf[256];
                sprintf(filenameBuf, "%s", fileName.c_str());
                int filenameLen = strlen(filenameBuf);
                filenameBuf[filenameLen - 3] = 'P';
                filenameBuf[filenameLen - 2] = 'K';
                filenameBuf[filenameLen - 1] = 'T';
                pktPath += filenameBuf;
                FILE *fPkt = fopen(pktPath.c_str(), "r");
                if (fPkt) {
                    while (!feof(fPkt)) {
                        char buf[256];
                        if (!fgets(buf, sizeof(buf), fPkt) || buf[0] == '#') {
                            continue;
                        }
                        fpc->AddPacket(buf);
                    }
                    fclose(fPkt);
                }

                continue;
            }
        }

        std::cerr << "Invalid object in test set: " << line << std::endl;
    }

    testSetStream.close();
    sLoadedTestSet = this;
    return true;
}

std::string TestSet::GetName()
{
    return testSetName;
}

double TestSet::GetStartTimeSec()
{
    return startTimeSec;
}

double TestSet::GetEndTimeSec()
{
    return endTimeSec;
}

double TestSet::GetStartTimeAutoRunSec()
{
    return startTimeAutoRunSec;
}

double TestSet::GetEndTimeAutoRunSec()
{
    return endTimeAutoRunSec;
}

int TestSet::GetCount()
{
    return flightPaths.size();
}

FlightPathConfig *TestSet::GetFlightPathAtIndex(int ix)
{
    if (ix < 0 || ix >= flightPaths.size()) {
        return nullptr;
    }
    return flightPaths[ix];
}

FlightPathConfig *TestSet::GetFlightPathByIdNr(uint32_t idNr)
{
    // TODO could use a map for faster access.
    for (int i = 0, imax = GetCount(); i < imax; i++) {
        FlightPathConfig *fpc = GetFlightPathAtIndex(i);
        if (fpc->GetIdNr() == idNr) {
            return fpc;
        }
    }
    return nullptr;
}

FlightPathConfig *TestSet::GetSelectedFlightPath()
{
    for (int i = 0, imax = GetCount(); i < imax; i++) {
        FlightPathConfig *fpc = GetFlightPathAtIndex(i);
        if (fpc->isSelected) {
            return fpc;
        }
    }
    return nullptr;
}
