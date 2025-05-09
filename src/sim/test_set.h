//
// test_set.h
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

#ifndef __TEST_SET_H__
#define __TEST_SET_H__

#include "flight_path_config.h"
#include <vector>

class TestSet {
public:
    static TestSet *LoadedTestSet();

    TestSet();
    virtual ~TestSet();

    bool LoadTestSet(const char *testSetPath);

    std::string GetName();

    double GetStartTimeSec();
    double GetEndTimeSec();

    double GetStartTimeAutoRunSec();
    double GetEndTimeAutoRunSec();

    int GetCount();
    FlightPathConfig *GetFlightPathAtIndex(int ix);
    FlightPathConfig *GetFlightPathByIdNr(uint32_t idNr);

    FlightPathConfig *GetSelectedFlightPath();

    double rotXZ = 60.0;
    double rotXY = -30.0;
    double trH = -0.2;
    double trV = -0.6;
    double camR = 0.2;

private:
    std::vector<FlightPathConfig*> flightPaths;

    std::string flightPathDirectory;
    std::string testSetName;
    double startTimeSec = 0.0;
    double endTimeSec = 0.0;

    double startTimeAutoRunSec = 0.0;
    double endTimeAutoRunSec = 200.0;
};

#endif // __TEST_SET_H__
