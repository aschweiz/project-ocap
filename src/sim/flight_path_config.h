//
// flight_path_config.h
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

#ifndef __FLIGHT_PATH_CONFIG_H__
#define __FLIGHT_PATH_CONFIG_H__

#include <string>
#include "flight_path.h"
#include "linalg.h"
#include "radio_link.h"
#include "predicted_trace.h"
#include "FlightPathExtrapolation.h"

#include "AlarmState.h"

using namespace linalg;

#define MAX_PACKET_LEN 64

typedef struct {
	double ts;
	int rssi;
	uint8_t data[MAX_PACKET_LEN];
} SPacket;

// A quadratic area.
class FlightPathConfig : IRadioLinkSubscriber
{
public:
	FlightPathConfig(std::string identifier);
	virtual ~FlightPathConfig();

	bool LoadFlp(const char *flpPath);
	bool LoadPflag(const char *pflagPath);

	void AddPacket(const char *packet);

	void SetTimeOffset(double startSec);
	void SetPositionOffset(double dLonDeg, double dLatDeg, double dAltMtr);

	std::string GetIdentifier();

	double GetLonMinDeg();
	double GetLonMaxDeg();
	double GetLatMinDeg();
	double GetLatMaxDeg();
	double GetAltMinMtr();
	double GetAltMaxMtr();

	int GetTimestampSecMin();
	int GetTimestampSecMax();

	// X=LON (Deg), Y=LAT (Deg), Z=ALT (Mtr)
	bool GetPositionDegDegMtrAtMillisec(long ms, Vector3d *vecLonLatAlt);

	bool GetSpeedAtSec(double timeSec, Vector3d *vel);

	void StartCollisionWarning();

	// Call this method at the end of each second.
	// Process all available data (own and received) and generate collision warnings.
	void YieldSimulationPrediction(long ms);

	// Call this method shortly after a new second has started.
	// Creates and transmits a data packet with our data.
	void YieldSimulationBroadcast(long ms);

	// IRadioLinkSubscription
  virtual int GetIdNr();
  virtual bool GetPosition(long ms, linalg::Vector3d *pos);
  virtual void OnReceiveMessage(RadioMessage &msg, double dbm);

	// Call to save a predicted path.
	virtual void SetPredictedTrace(PredictedTrace *pt);
	virtual int GetNofPredictedTraces();
	virtual PredictedTrace *GetPredictedTraceAtIndex(int ix);

	// Alarm level for each second of the flight, with respect to the selected FPC.
	void SetAlarmLevelAtSecond(int sec, EAlarmLevel level);
	EAlarmLevel GetAlarmLevelAtSecond(int sec);

	// RSSI for each second of the flight, with respect to the selected FPC.
	void SetRssiAtSecond(int sec, int rssi);
	int GetRssiAtSecond(int sec);

	bool HasPackets();

public:
	bool isSelected = false;
	std::string alarmMessage;
	EAlarmLevel alarmLevel; // 0 = none, 1 = low, 3 = high
	int alarmIntensity;

	// Displaying insights into the OCAP algorithm.
	bool hasZVec = false;
	linalg::Vector3d zVec;
	linalg::Vector3d posVec;
	linalg::Vector3d velVec;
	EOcapPathModel predictionModel;

private:
	bool CalculatePosAndVel(long ms, linalg::Vector3d *posMtr, linalg::Vector3d *velMtrSec);

private:
	FlightPath *flightPath = nullptr;
	std::string identifier;
	int idNr = 0;

	double timeOffsetSec;
	linalg::Vector3d posOffsetLonLatMtr;

	std::vector<PredictedTrace*> predictedTraces;

	std::map<int, int> rssiAtSecond;       // Vs. selected fpc
	std::map<int, int> alarmLevelAtSecond; // Vs. selected fpc

	bool hasPackets = false;               // Vs. selected fpc
	long packetStartSecond = __LONG_MAX__;
	std::map<long, SPacket> packetAtSecond;
};

#endif // __FLIGHT_PATH_CONFIG_H__
