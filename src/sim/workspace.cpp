
//
// workspace.cpp
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

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <iostream>
#include <map>

#include "radio_link.h"
#include "workspace.h"

#include "flight_path_config.h"
#include "linalg.h"
#include "predicted_trace.h"
#include "test_set.h"
#include "time_range.h"
#include "world_coords.h"

#include "win_objects.h"
#include "win_time.h"
#include "win_world.h"

#include "AlarmStateList.h"
#include "CalculateOtherData.h"
#include "CalculateOwnData.h"
#include "FlightObjectList.h"
#include "FlightObjectOrientation.h"
#include "FlightPathExtrapolation.h"
#include "Prediction.h"

using namespace linalg;
using namespace gui;

// From Prediction.c
extern TVector sOwnFlightPath[T_MAX_SEC];
extern TVector *sOtherFlightPath; // [20][T_MAX_SEC];
extern uint32_t *sOtherFlightPathIdNr; // [20];

static Workspace *sWorkspace;

WorldCoords worldCoords;
TestSet *testSet = nullptr;

TimeRange tr;

static void VectorTest(void);
static void IgcTest(void);

Workspace *Workspace::Instance()
{
    if (!sWorkspace) {
        sWorkspace = new Workspace();
    }
    return sWorkspace;
}

Workspace::Workspace()
{
    sOtherFlightPath = new TVector[FLIGHT_OBJECT_LIST_LENGTH * T_MAX_SEC];
    sOtherFlightPathIdNr = new uint32_t[FLIGHT_OBJECT_LIST_LENGTH];
}

Workspace::~Workspace()
{
    delete[] sOtherFlightPath;
    delete[] sOtherFlightPathIdNr;
    sOtherFlightPath = nullptr;
    sOtherFlightPathIdNr = nullptr;
}

void Workspace::Initialize(std::string pathToTestSet, bool isAutoRun, bool isRealTime, std::string resultFile)
{
    this->isAutoRun = isAutoRun;
    this->isRealTime = isRealTime;
    collisionResultFileName = resultFile;

    // Load simulation data.
    testSet = new TestSet();
    bool testSetLoaded = testSet->LoadTestSet(pathToTestSet.c_str());

    double lonMinDeg = 181; // x
    double lonMaxDeg = -181;
    double latMinDeg = 91; // y
    double latMaxDeg = -91;
    double altMinMtr = 999999;
    for (int i = 0; i < testSet->GetCount(); i++) {
        FlightPathConfig *fpc = testSet->GetFlightPathAtIndex(i);
        lonMinDeg = std::min(lonMinDeg, fpc->GetLonMinDeg());
        lonMaxDeg = std::max(lonMaxDeg, fpc->GetLonMaxDeg());
        latMinDeg = std::min(latMinDeg, fpc->GetLatMinDeg());
        latMaxDeg = std::max(latMaxDeg, fpc->GetLatMaxDeg());
        altMinMtr = std::min(altMinMtr, fpc->GetAltMinMtr());
    }

    tr.SetStartEnd(testSet->GetStartTimeSec(), testSet->GetEndTimeSec());

    // Create the world coordinates.
    worldCoords.setLonLatAltRanges(lonMinDeg, lonMaxDeg, latMinDeg, latMaxDeg, altMinMtr);

    // Simulation window
    Win_World *winWorld = Win_World::Instance();
    winWorld->SetWorldCoords(&worldCoords);
    winWorld->SetTestSet(testSet);
    winWorld->SetTimeRange(&tr);

    // Time window
    Win_Time *winTime = Win_Time::Instance();
    winTime->SetTestSet(testSet);

    tr.SetFrom(0, false);
    tr.SetTo(100, false);
    winTime->SetTimeRange(&tr);

    // Objects window
    Win_Objects *winObjects = Win_Objects::Instance();
    winObjects->SetTestSet(testSet);
    winObjects->SetTimeRange(&tr);
}

void Workspace::PostRedisplayAll()
{
    Win_Time::Instance()->PostRedisplay();
    Win_World::Instance()->PostRedisplay();
    Win_Objects::Instance()->PostRedisplay();
}

bool Workspace::IsAutoRun()
{
    return isAutoRun;
}

bool Workspace::IsRealTime()
{
    return isRealTime;
}

void Workspace::StartCollisionWarning()
{
    if (collisionWarningRunning) {
        return;
    }
    collisionWarningRunning = true;

    printf("--- StartCollisionWarning\n");
    collisionWarningLastTimeMs = 0;

    // Configure the prediction logic.

    // Double width at end of extrapolation; double and 4x width for L2 and L1.
    predictionInit(30, 2, 4);

    // Create the result file.

    collisionResultFile = fopen(collisionResultFileName.c_str(), "w");
    fprintf(collisionResultFile, "test set                    = %s\n", testSet->GetName().c_str());
    FlightPathConfig *fpcSelected = testSet->GetSelectedFlightPath();
    fprintf(collisionResultFile, "selected aircraft           = %s\n",
        !fpcSelected ? "none" : fpcSelected->GetIdentifier().c_str());

    // Let the selected aircraft start the collision warning algorithm.
    FlightPathConfig *fpc = testSet->GetSelectedFlightPath();
    if (fpc) {
        fpc->StartCollisionWarning();
    }
}

void Workspace::StopCollisionWarning()
{
    printf("--- StopCollisionWarning\n");
    collisionWarningRunning = false;

    if (collisionResultFile) {
        fclose(collisionResultFile);
        collisionResultFile = nullptr;
    }
}

bool Workspace::IsCollisionWarningRunning()
{
    return collisionWarningRunning;
}

void Workspace::YieldCollisionWarning(long timeMs)
{
    if (!collisionWarningRunning) {
        return;
    }
    if (timeMs <= collisionWarningLastTimeMs) {
        return;
    }

    printf("--- YieldCollisionWarning %ld\n", timeMs);

    int oldSec = (int)(collisionWarningLastTimeMs / 1000);
    int newSec = (int)(timeMs / 1000);

    collisionWarningLastTimeMs = timeMs;

    if (oldSec == newSec) {
        return;
    }

    // Distribute the last second's packets to the subscribers.
    long oldMsEnd = (long)oldSec * 1000 + 999;
    printf("--- YieldDistribution %ld\n", oldMsEnd);
    RadioLink::Instance()->YieldDistribution(oldMsEnd);

    // For development only
    for (int i = 0; i < FLIGHT_OBJECT_LIST_LENGTH; i++) {
        sOtherFlightPathIdNr[i] = 0;
    }

    // Clear alarm message on all aircraft.
    // The selected aircraft will set alarm messages if necessary.
    for (int i = 0; i < testSet->GetCount(); i++) {
        FlightPathConfig *fpc = testSet->GetFlightPathAtIndex(i);
        fpc->alarmMessage = "";
        fpc->alarmLevel = ALARM_LEVEL_NONE;
    }

    // Run prediction for selected aircraft, broadcast packets for all aircraft.
    for (int i = 0; i < testSet->GetCount(); i++) {
        FlightPathConfig *fpc = testSet->GetFlightPathAtIndex(i);

        if (fpc->isSelected) {
            // Selected aircraft: Run the prediction.
            printf("--- YieldSimulationPrediction %s %ld\n", fpc->GetIdentifier().c_str(), oldMsEnd);
            fpc->YieldSimulationPrediction(oldMsEnd);

            // Store predicted path for selected aircraft.
            PredictedTrace *pt = new PredictedTrace();
            pt->identifier = fpc->GetIdentifier();
            pt->idNr = fpc->GetIdNr();
            pt->startTimeMs = timeMs;
            pt->isSelected = true;
            for (int i = 0; i < T_MAX_SEC; i++) {
                pt->positionsMtr[i].Set(sOwnFlightPath[i].x, sOwnFlightPath[i].y, sOwnFlightPath[i].z);
            }
            fpc->SetPredictedTrace(pt);
        }

        printf("--- YieldSimulationBroadcast %s %ld\n", fpc->GetIdentifier().c_str(), timeMs);
        fpc->YieldSimulationBroadcast(timeMs);
    }

    // Set the alarm level for every aircraft
    // and copy the highest alarm level to the selected aircraft.
    FlightPathConfig *fpcSelected = testSet->GetSelectedFlightPath();
    if (fpcSelected) {
        for (int i = 0; i < testSet->GetCount(); i++) {
            FlightPathConfig *fpc = testSet->GetFlightPathAtIndex(i);
            fpc->SetAlarmLevelAtSecond(newSec, fpc->alarmLevel);
            if (fpc->isSelected) {
                continue;
            }
            if (fpc->alarmLevel > fpcSelected->GetAlarmLevelAtSecond(newSec)) {
                fpcSelected->SetAlarmLevelAtSecond(newSec, fpc->alarmLevel);
            }
        }
    }

    // Log alarm message on all aircraft.
    for (int i = 0; i < testSet->GetCount(); i++) {
        FlightPathConfig *fpc = testSet->GetFlightPathAtIndex(i);
        if (fpc->alarmLevel == ALARM_LEVEL_NONE) {
            continue;
        }
        if (collisionResultFile) {
            fprintf(collisionResultFile, "%7d;%s;%d;%s;%d\n",
                newSec, fpc->GetIdentifier().c_str(), (int)fpc->alarmLevel,
                fpc->alarmMessage.c_str(), fpc->alarmIntensity);
        }
    }

    // Store predicted path for non-selected aircraft.
    for (int i = 0; i < FLIGHT_OBJECT_LIST_LENGTH; i++) {
        if (!sOtherFlightPathIdNr[i]) {
            continue;
        }
        uint32_t curIdNr = sOtherFlightPathIdNr[i];
        FlightPathConfig *fpc = testSet->GetFlightPathByIdNr(curIdNr);
        if (!fpc) {
            continue;
        }
        PredictedTrace *pt = new PredictedTrace();
        pt->identifier = fpc->GetIdentifier();
        pt->idNr = curIdNr;
        pt->startTimeMs = timeMs;
        pt->isSelected = true;
        int baseOffset = T_MAX_SEC * i;
        for (int t = 0; t < T_MAX_SEC; t++) {
            pt->positionsMtr[t].Set(
                sOtherFlightPath[baseOffset + t].x,
                sOtherFlightPath[baseOffset + t].y,
                sOtherFlightPath[baseOffset + t].z);
        }
        fpc->SetPredictedTrace(pt);
    }
}

FlightPathConfig *Workspace::GetFlightPathByIdNr(uint32_t idNr)
{
    return testSet->GetFlightPathByIdNr(idNr);
}

void Workspace::SetReplayPackets(bool r)
{
    isReplayPackets = r;
    Win_Time::Instance()->PostRedisplay();
}

bool Workspace::GetReplayPackets()
{
    return isReplayPackets;
}

void Workspace::SetUseIConspicuity2(bool r)
{
    isIConspicuity2 = r;
    Win_Time::Instance()->PostRedisplay();
}

bool Workspace::GetUseIConspicuity2()
{
    return isIConspicuity2;
}
