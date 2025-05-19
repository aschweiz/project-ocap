//
// workspace.h
//
// OCAP - Open Collision Avoidance Protocol
//
// Simulation environment.
//
// 04.07.2024 ASR  First version
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

#ifndef __WORKSPACE_H__
#define __WORKSPACE_H__ 1

#include "flight_path_config.h"
#include <stdio.h>
#include <vector>

class Workspace {
public:
    static Workspace *Instance();

private:
    Workspace();

public:
    ~Workspace();

    void Initialize(std::string pathToTestSet, bool isAutoRun, bool isRealTime, std::string resultFile);

    void PostRedisplayAll();

    bool IsAutoRun();
    bool IsRealTime();

    // Call at the start of a collision warning simulation.
    void StartCollisionWarning();
    void StopCollisionWarning();
    bool IsCollisionWarningRunning();

    // Call for every simulation step.
    void YieldCollisionWarning(long timeMs);

    FlightPathConfig *GetFlightPathByIdNr(uint32_t idNr);

    void SetReplayPackets(bool r);
    bool GetReplayPackets();
    void SetUseIConspicuity2(bool r);
    bool GetUseIConspicuity2();

private:
    bool isAutoRun = false;
    bool isRealTime = false;
    bool isReplayPackets = false;
    bool isIConspicuity2 = false;

    bool collisionWarningRunning = false;
    long collisionWarningLastTimeMs = 0;
    
    std::string collisionResultFileName;
    FILE *collisionResultFile = nullptr;
};

#endif // __WORKSPACE_H__
