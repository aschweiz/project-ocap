//
// flight_path_config.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Wrapper around a single flight path for using it in the OCAP simulation.
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

#include "flight_path_config.h"
#include "ads_l_integration.h"
#include "ads_l_xxtea.h"
#include "lon_lat_util.h"
#include "predicted_trace.h"
#include "radio_link.h"
#include "test_set.h"
#include "workspace.h"

#include "AlarmService.h"
#include "AlarmStateList.h"
#include "CalculateOtherData.h"
#include "CalculateOwnData.h"
#include "Configuration.h"
#include "FlightObjectList.h"
#include "FlightObjectOrientation.h"
#include "FlightPathExtrapolation.h"
#include "Prediction.h"

#define USE_ALARM_SERVICE 1

#define ALARM_STATE_INFO_BUF_SIZE 256

static int sNextIdNr = 1;

// From Prediction.c
extern TVector sOwnFlightPath[T_MAX_SEC];
extern TVector *sOtherFlightPath; // [20][T_MAX_SEC];
extern uint32_t *sOtherFlightPathIdNr; // [20];

static float convert_latDegN_E5_toMtr(int32_t latDegN_E5);
static float convert_lonDegE_E5_toMtr(int32_t lonDegE_E5, int32_t latDegN_E5);

static float convert_latMtr_to_degN_E5(int32_t latMtr);
static float convert_lonMtr_toDegE_E5(int32_t lonMtr, int32_t latDegN_E5);

static int hexCharToDec(char c);

static void printPacketData(const uint8_t *packetData, int packetLength);


FlightPathConfig::FlightPathConfig(std::string i)
{
    flightPath = new FlightPath();
    identifier = i;
    idNr = sNextIdNr++;

    RadioLink::Instance()->Subscribe(this);
}

FlightPathConfig::~FlightPathConfig()
{
}

bool FlightPathConfig::LoadFlp(const char *flpPath)
{
    return flightPath->LoadFile(flpPath, FLIGHT_PATH_FILE_TYPE_FLP);
}

bool FlightPathConfig::LoadPflag(const char *flpPath)
{
    return flightPath->LoadFile(flpPath, FLIGHT_PATH_FILE_TYPE_PFLAG);
}

void FlightPathConfig::AddPacket(const char *packetLine)
{
    hasPackets = true;

    // Try to parse the log entry.
    // Examples:
    // TEST,1740233083.969,0,ADS-L,TX,1C006903745202B77605B405...C035CF51
    double ts;
    int rssi;
    char packetChars[1024];
    int nofScanned = sscanf(packetLine, "$TEST,%lf,%d,ADS-L,TX,%[^,],", &ts, &rssi, &packetChars[0]);
    if (nofScanned < 3 || (int)ts == 0) {
        return;
    }
    // TX side
    rssi = 13;

    // Derive the packet content from the parsed log entry.
    long tsInt = (long)ts;
    SPacket packet;
    packet.ts = ts;
    packet.rssi = rssi;

    // First byte is the length of the packet, in bytes.
    const char *packetChar = &packetChars[0];
    int packetLength = hexCharToDec(*packetChar++);
    packetLength = (packetLength << 4) | hexCharToDec(*packetChar++);
    packet.data[0] = (uint8_t)packetLength;

    // Remaining bytes contain the packet content.
    for (int i = 1; i < packetLength; i++) {
        int v = hexCharToDec(*packetChar++);
        v = (v << 4) | hexCharToDec(*packetChar++);
        packet.data[i] = v;
    }

    // XXTEA decode; 5 x 32-bit, even with iConspicuity2.
    printf("XXTEA-decoding replay packet: ");
    printPacketData(packet.data, packetLength);
    adslXxteaDecodeWithPubkey((uint32_t*)&packet.data[2], 5);
    printf(" --> ");
    printPacketData(packet.data, packetLength);
    printf("\n");

    packetAtSecond[ts] = packet;
    if (ts < packetStartSecond) {
        packetStartSecond = ts;
    }
}

void FlightPathConfig::SetTimeOffset(double startSec)
{
    timeOffsetSec = startSec;
}

void FlightPathConfig::SetPositionOffset(double dLonDeg, double dLatDeg, double dAltMtr)
{
    posOffsetLonLatMtr.Set(dLonDeg, dLatDeg, dAltMtr);
}

bool FlightPathConfig::GetSpeedAtSec(double timeSec, Vector3d *vel)
{
    return flightPath->GetVelocityAtMillisec(1000 * timeSec, vel);
}

std::string FlightPathConfig::GetIdentifier()
{
    return identifier;
}

double FlightPathConfig::GetLonMinDeg()
{
    return flightPath->GetLonMinDeg() + posOffsetLonLatMtr.X();
}

double FlightPathConfig::GetLonMaxDeg()
{
    return flightPath->GetLonMaxDeg() + posOffsetLonLatMtr.X();
}

double FlightPathConfig::GetLatMinDeg()
{
    return flightPath->GetLatMinDeg() + posOffsetLonLatMtr.Y();
}

double FlightPathConfig::GetLatMaxDeg()
{
    return flightPath->GetLatMaxDeg() + posOffsetLonLatMtr.Y();
}

double FlightPathConfig::GetAltMinMtr()
{
    return flightPath->GetAltMinMtr() + posOffsetLonLatMtr.Z();
}

double FlightPathConfig::GetAltMaxMtr()
{
    return flightPath->GetAltMaxMtr() + posOffsetLonLatMtr.Z();
}

int FlightPathConfig::GetTimestampSecMin()
{
    return flightPath->GetTimestampSecMin() + timeOffsetSec;
}

int FlightPathConfig::GetTimestampSecMax()
{
    return flightPath->GetTimestampSecMax() + timeOffsetSec;
}

// X=LON (deg), Y=LAT (deg), Z=ALT (mtr)
bool FlightPathConfig::GetPositionDegDegMtrAtMillisec(long ms, Vector3d *vecLonLatAlt)
{
    linalg::Vector3d v;

    bool result = flightPath->GetPositionDegDegMtrAtMillisec(ms + timeOffsetSec * 1000, &v);
    vecLonLatAlt->Set(
        v.X() + posOffsetLonLatMtr.X(), // lonDeg
        v.Y() + posOffsetLonLatMtr.Y(), // latDeg
        v.Z() + posOffsetLonLatMtr.Z()); // mtrDeg

    return result;
}

void FlightPathConfig::StartCollisionWarning()
{
    // Create our flight object and register for radio packets.

    flightObjectListInit(idNr);
}

// Call this method at the end of each second.
// Process all available data (own and received) and generate collision warnings.
void FlightPathConfig::YieldSimulationPrediction(long ms)
{
    if (ms < 1000) {
        return;
    }

    printf("----- YieldSimulationPrediction\n");
    int ts = ms / 1000;

    // Radio messages have already been received and applied.
    // Apply our own data, then run the collision warning algorithm.

    Vector3d posNewMtr, velNewMtrSec; // lonMtr,latMtr,altMtr
    bool hasPosVel = CalculatePosAndVel(ms, &posNewMtr, &velNewMtrSec);
    if (!hasPosVel) {
        return;
    }

    calculateOwnDataFromGpsInfo(ts, posNewMtr.GetVector(), velNewMtrSec.GetVector());

    // Perform collision prediction.

    predictionCalculateAlarmStates(ts);

    // Show the result.

    alarmStateListDump();

    TFlightObjectOwn *fOwn = flightObjectListGetOwn();
    int nofAlarmStates = alarmStateListGetCount();

    // Provide the most critical alarm (at the head of the list) to the alarm service.
    TAlarmState *theA = NULL;
    if (nofAlarmStates > 0) {
        theA = alarmStateListGetAtIndex(0);
    }

    // Prepare detailed information about the alarm state.
    char buf[ALARM_STATE_INFO_BUF_SIZE] = { 0 };

	// Calculate the distance and orientation to the "offending" aircraft.
	TFlightObjectOrientation fo;
	int hasOrientation = 0;
    TVector vectorToOtherMemory, *vectorToOther = &vectorToOtherMemory;
	int distMtr = -1;
    TFlightObjectOther *fOther = NULL;
    if (theA) {
		// If an alarm state is available, we can directly use the dist vector.
		fOther = theA->flightObject;
		vectorToOther = &theA->curDistanceToFlightObject;
    } else {
		// No alarm state, so calculate the dist vector based on previous aircraft.
		TAlarmServiceEntry *eOld = alarmServiceGetMostCritical();
		if (eOld && eOld->flightObject) {
			fOther = eOld->flightObject;
			vectorCopy(vectorToOther, &fOther->rxPos);
			vectorSubtractVector(vectorToOther, &fOwn->rxPos);
		}
    }
	// Calculate the distance based on the vector to the other aircraft.
	if (vectorToOther) {
		hasOrientation = flightObjectOrientationCalculate(
				&fo, vectorToOther, &fOwn->rxVel);
		distMtr = hasOrientation ? fo.distanceMeters : -1;
	}

    // Print alarm information.
    if (theA) {
        if (hasOrientation) {
            snprintf(buf, ALARM_STATE_INFO_BUF_SIZE, "L%d h=%d° d=%dm dt=%ds",
                theA->level, fo.directionDeg, fo.distanceMeters, theA->timeToEncounterSec);
        } else {
            snprintf(buf, ALARM_STATE_INFO_BUF_SIZE, "L%d h=N/A d=%dm dt=%ds",
                theA->level, fo.distanceMeters, theA->timeToEncounterSec);
        }
        printf("ALARM: %06x  %s\n", fOther->id, buf);
    }

#ifdef USE_ALARM_SERVICE
    // Reflect the alarm level on the flight path.
    // An alarm service entry may exist even if there was no alarm state in this round.
    alarmServiceUpdateMostCritical(theA, distMtr);
    TAlarmServiceEntry *e = alarmServiceGetMostCritical();
    if (e && e->alarmState) {
        FlightPathConfig *fpcOther = Workspace::Instance()->GetFlightPathByIdNr(
            e->alarmState->flightObject->id);
        if (fpcOther) {
            fpcOther->alarmMessage = buf;
            fpcOther->alarmLevel = alarmServiceGetLevel(e); // downscaled 0-3
            fpcOther->alarmIntensity = e->level; // 0-12, filtered by AlarmService
        }
    }
#else
    if (theA) {
        FlightPathConfig *fpcOther = Workspace::Instance()->GetFlightPathByIdNr(theA->flightObject->id);
        fpcOther->alarmMessage = buf;
        fpcOther->alarmLevel = theA->level;
        fpcOther->alarmIntensity = (int)theA->level;
    }
#endif
}

// Call this method shortly after a new second has started.
// Creates and transmits a data packet with the data for this flight path.
void FlightPathConfig::YieldSimulationBroadcast(long ms)
{
    printf("----- YieldSimulationBroadcast\n");

    int s = ms / 1000;
    int msOffset = rand() % 600; // s.200 ... s.799
    long txMs = (long)s * 1000 + 200 + msOffset;

    Vector3d posNewMtr, velNewMtrSec;
    bool hasPosVel = CalculatePosAndVel(ms, &posNewMtr, &velNewMtrSec);
    if (!hasPosVel) {
        return;
    }

    RadioMessage msg;
    msg.sender = this;

    printf("Radio message TX: %x p=(%.2f, %.2f, %2.f), v=(%.2f, %.2f, %.2f)\n",
        msg.sender->GetIdNr(),
        (float)posNewMtr.X(), (float)posNewMtr.Y(), (float)posNewMtr.Z(),
        (float)velNewMtrSec.X(), (float)velNewMtrSec.Y(), (float)velNewMtrSec.Z());

    TFlightObjectOwn *f = flightObjectListGetOwn();
    float hasZVec = false;
    if (f->z_i_ctr > 0) {
        TVector rzVec;
        vectorCopy(&rzVec, &f->z_avg);
        vectorSubtractVector(&rzVec, &f->rxPos);
        // r_z_vec points from the new position (ri) to Z.
        printf("                        dz=(%.2f, %.2f, %2.f) - ", rzVec.x, rzVec.y, rzVec.z);
        hasZVec = true;
        predictionModel = f->pathModel;
        if (predictionModel == OCAP_PATH_MODEL_ARC) {
            printf("ARC\n");
        } else if (predictionModel == OCAP_PATH_MODEL_LINEAR) {
            printf("LINEAR\n");
        } else if (predictionModel == OCAP_PATH_MODEL_SPHERIC) {
            printf("SPHERIC\n");
        } else {
            printf("???\n");
        }
        vectorCopy(zVec.GetVector(), &f->z_avg);
        vectorCopy(posVec.GetVector(), &f->rxPos);
        vectorCopy(velVec.GetVector(), &f->rxVel);
    }

    //
    // Create a new, freshly generated message?

    if (!hasPackets || !Workspace::Instance()->GetReplayPackets()) {

        // If enabled, we generate iConspicuity2 packets with the Z vector.
        Vector3d z, *zPtr = nullptr;
        if (hasZVec && Workspace::Instance()->GetUseIConspicuity2()) {
            z = zVec - posNewMtr;
            zPtr = &z;
        }

        bool hasFreshMessage = CreateRadioMessage(
            &msg, GetIdNr(), txMs, posNewMtr, velNewMtrSec,
            zPtr, predictionModel);
        if (hasFreshMessage) {
            RadioLink::Instance()->Broadcast(msg);
        }

        return;
    }

    //
    // Replaying recorded data packet.

    // Do we have a recorded data packet for the current point in time?
    long sec = packetStartSecond + s;
    bool hasOldMessage = packetAtSecond.find(sec) != packetAtSecond.end();
    if (!hasOldMessage) {
        return;
    }

    // We do have a recorded data packet. Copy it into the message object and log it.
    SPacket &packet = packetAtSecond[sec];
    uint8_t *packetData = &packet.data[0];
    int packetLength = packetData[0]; // First byte is always the length, in bytes.
    for (int i = 0; i < packetLength; i++) {
        msg.bytes[i] = packetData[i];
    }
    printf("Replaying packet: ");
    printPacketData(packetData, packetLength);
    printf("\n");

    // Fill some other fields of the message not available from the recorded packet.
    msg.txStartTimeMs = txMs; // s.200 ... s.799
    msg.txDurationMs = 5.0;

    RadioLink::Instance()->Broadcast(msg);
}

int FlightPathConfig::GetIdNr()
{
    return idNr;
}

bool FlightPathConfig::GetPosition(long ms, linalg::Vector3d *pos)
{
    return GetPositionDegDegMtrAtMillisec(ms, pos);
}

void FlightPathConfig::OnReceiveMessage(RadioMessage &msg, double dbm)
{
    if (!isSelected) {
        return;
    }

    printf("----- OnReceiveMessage %s <- %d\n", GetIdentifier().c_str(), msg.sender->GetIdNr());
    int otherIdNr = msg.sender->GetIdNr();
    TFlightObjectOther *fOther = nullptr;

    // If we already know this object, we take it from the list of known objects.
    for (int i = 0; i < flightObjectListGetOtherCount(); i++) {
        TFlightObjectOther *fCur = flightObjectListGetOtherAtIndex(i);
        if (fCur->id == otherIdNr) {
            fOther = fCur;
            break;
        }
    }

    // If we don't know the object yet, we add it to the list.
    if (!fOther) {
        printf("adding %d\n", otherIdNr);
        fOther = flightObjectListAddOther(otherIdNr);
    }

    Vector3d posMtr, velMtrSec;
    Vector3d zMtr;
    bool hasZ = false;
    EOcapPathModel pathModel = OCAP_PATH_MODEL_LINEAR;
    bool isMessageValid = DecodeRadioMessage(
        msg, posMtr, velMtrSec,
        &hasZ, zMtr, &pathModel // iConspicuity2 includes Z and pathModel
    );
    if (!isMessageValid) {
        return;
    }

    printf("Radio message RX: %x p=(%.2f, %.2f, %2.f), v=(%.2f, %.2f, %.2f)\n",
        msg.sender->GetIdNr(),
        posMtr.X(), posMtr.Y(), posMtr.Z(),
        velMtrSec.X(), velMtrSec.Y(), velMtrSec.Z());

    // Show additional information about the r-z vector, if available.
    if (hasZ) {
        printf("                        dz=(%.2f, %.2f, %2.f) - ",
            zMtr.X(), zMtr.Y(), zMtr.Z());
        hasZVec = true;
        if (pathModel == OCAP_PATH_MODEL_ARC) {
            printf("ARC\n");
        } else if (pathModel == OCAP_PATH_MODEL_LINEAR) {
            printf("LINEAR\n");
        } else if (pathModel == OCAP_PATH_MODEL_SPHERIC) {
            printf("SPHERIC\n");
        } else {
            printf("???\n");
        }
    }

    TestSet *ts = TestSet::LoadedTestSet();
    if (ts) {
        FlightPathConfig *fpcOther = ts->GetFlightPathByIdNr(fOther->id);
        int packetRssi = 0; // TODO
        fpcOther->SetRssiAtSecond(msg.txStartTimeMs / 1000, packetRssi);
    }

    // We update the data for this object.
    calculateOtherDataFromInfo(
        fOther,
        msg.txStartTimeMs / 1000,
        posMtr.GetVector(),
        velMtrSec.GetVector(),
        hasZ ? zMtr.GetVector() : nullptr, pathModel);
}

bool FlightPathConfig::CalculatePosAndVel(
    long ms, linalg::Vector3d *posMtr, linalg::Vector3d *velMtrSec)
{
    int s = ms / 1000;

    Vector3d posCurDeg; // lonDeg,latDeg,altMtr
    bool hasPosCur = GetPositionDegDegMtrAtMillisec((long)s * 1000, &posCurDeg);
    if (!hasPosCur) {
        return false;
    }

    Vector3d posCurMtr; // lonMtr,latMtr,altMtr
    lonDegLatDegAltMtrToMtrMtrMtr(posCurDeg.GetVector(), posCurMtr.GetVector());

    Vector3d velCurMtrSec;
    bool hasVelCur = GetSpeedAtSec(s, &velCurMtrSec);

    *posMtr = posCurMtr;
    *velMtrSec = velCurMtrSec;
    return true;
}

void FlightPathConfig::SetPredictedTrace(PredictedTrace *pt)
{
    predictedTraces.push_back(pt);
}

int FlightPathConfig::GetNofPredictedTraces()
{
    return predictedTraces.size();
}

PredictedTrace *FlightPathConfig::GetPredictedTraceAtIndex(int ix)
{
    return predictedTraces[ix];
}

// Alarm level for each second of the flight, with respect to the selected FPC.
void FlightPathConfig::SetAlarmLevelAtSecond(int sec, EAlarmLevel level)
{
    alarmLevelAtSecond[sec] = (int)level;
}

EAlarmLevel FlightPathConfig::GetAlarmLevelAtSecond(int sec)
{
    if (alarmLevelAtSecond.find(sec) == alarmLevelAtSecond.end()) {
        return ALARM_LEVEL_NONE;
    }
    return (EAlarmLevel)alarmLevelAtSecond[sec];
}

// RSSI for each second of the flight, with respect to the selected FPC.
void FlightPathConfig::SetRssiAtSecond(int sec, int rssi)
{
    rssiAtSecond[sec] = rssi;
}

int FlightPathConfig::GetRssiAtSecond(int sec)
{
    if (rssiAtSecond.find(sec) == rssiAtSecond.end()) {
        return -999;
    }
    return rssiAtSecond[sec];
}

bool FlightPathConfig::HasPackets()
{
    return hasPackets;
}

static int hexCharToDec(char c)
{
    if (c >= '0' && c <= '9') {
        return (int)c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return (int)c + 10 - 'A';
    }
    if (c >= 'a' && c <= 'f') {
        return (int)c + 10 - 'a';
    }
    return 0;
}

static void printPacketData(const uint8_t *packetData, int packetLength)
{
    for (int i = 0; i < packetLength; i++) {
        printf("%02X", packetData[i]);
    }
}
