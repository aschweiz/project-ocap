//
// AlarmService.c
//
// Alarm state filtering.
//
// 26.03.2025 ASR  First version.
//
// Software License (BSD):
// Copyright 2025 Classy Code GmbH.
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

#include "OcapLog.h"
#include "AlarmService.h"

#define FAR_AWAY_METERS 1000000

// We stretch the alarm level (low=1, high=3) internally by this factor
// so that we can apply a fine-grained decay.
#define ALARM_SERVICE_FACTOR 4

// For the moment, we maintain only 1 entry, for the most critical alarm.
// A future extension would be to maintain multiple entries and provide
// the most critical one to the client.
static TAlarmServiceEntry sEntry;


static void alarmServiceUpdateDataFields(
	TAlarmServiceEntry *e, int targetLevel, int distMtr, int timeToEncounterSec);


void alarmServiceUpdateMostCritical(TAlarmState *a, int distMtr)
{
	// No alarm state: Decay slowly towards level 0.
	if (!a) {
		alarmServiceUpdateDataFields(&sEntry, 0, -1, -1);
		return;
	}
	// Convert alarm state levels (1 to 3) to our extended range.
	int targetLevel = (int)a->level * ALARM_SERVICE_FACTOR;
	int timeToEncounterSec = a->timeToEncounterSec;
	if (a->flightObject && a->flightObject != sEntry.flightObject) {
		// A new aircraft causes the alarm.
		// Remember the aircraft.
		sEntry.flightObject = a->flightObject;
		sEntry.distMtr = FAR_AWAY_METERS;
		sEntry.approaching = 1;
	}
	alarmServiceUpdateDataFields(&sEntry, targetLevel, distMtr, timeToEncounterSec);
}

TAlarmServiceEntry *alarmServiceGetMostCritical(void)
{
	return &sEntry;
}

EAlarmLevel alarmServiceGetLevel(TAlarmServiceEntry *e)
{
	if (!e || e->level <= 0) {
		return ALARM_LEVEL_NONE;
	}
	// No alarm if the distance between us and the aircraft is increasing.
	if (!e->approaching) {
		return ALARM_LEVEL_NONE;
	}
	// Rounds towards the higher alarm level.
	return (EAlarmLevel)((e->level + ALARM_SERVICE_FACTOR - 1) / ALARM_SERVICE_FACTOR);
}


static void alarmServiceUpdateDataFields(
	TAlarmServiceEntry *e, int targetLevel, int distMtr, int timeToEncounterSec)
{
	// Update our alarm level.
	int delta = targetLevel - e->level;
	if (delta == 0) {
		// NOP
	} else if (delta > 0) {
		// Immediately boost to higher target level.
		e->level = targetLevel;
	} else {
		// Slowly decay towards the target level.
		if (-delta > ALARM_SERVICE_FACTOR) {
			delta = -2;
		} else {
			delta = -1;
		}
		e->level += delta;
	}
	// Approaching or flying away?
	if (distMtr >= 0) {
		e->approaching = (distMtr < e->distMtr) ? 1 : 0;
		e->distMtr = distMtr;
	}
	// Update time to encounter, if available.
	if (timeToEncounterSec >= 0) {
		e->timeToEncounterSec = timeToEncounterSec;
	} else if (e->approaching) {
		// The aircraft is approaching and we don't have new information,
		// so we count down to maintain continuity.
		e->timeToEncounterSec--;
	}
	// Drop after complete decay.
	if (e->level <= 0) {
		e->level = 0;
		e->flightObject = 0L;
	}
}
