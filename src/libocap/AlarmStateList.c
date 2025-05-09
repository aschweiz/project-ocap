//
// AlarmStateList.c
//
// OCAP - Open Collision Avoidance Protocol
//
// Alarm state list information.
//
// 02.07.2024 ASR  First version.
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

#include "OcapLog.h"
#include "AlarmStateList.h"

// Calculating this value ad-hoc will save space but increase computing time on insert.
#define ALARM_STATE_PRIO(l, t) (100 * (int)l + (T_MAX_SEC - t))


static TAlarmState sAlarmStates[ALARM_STATE_LIST_LENGTH];
static int sAlarmStateOrder[ALARM_STATE_LIST_LENGTH];
static int sAlarmStateSize;


static void alarmStateListSort(void);


void alarmStateListClear(void)
{
	sAlarmStateSize = 0;
	for (int i = 0; i < ALARM_STATE_LIST_LENGTH; i++) {
		sAlarmStateOrder[i] = -1;
	}
}

int alarmStateListGetCount(void)
{
	return sAlarmStateSize;
}

TAlarmState *alarmStateListGetAtIndex(int ix)
{
	int o = sAlarmStateOrder[ix];
	TAlarmState *a = &sAlarmStates[o];
	return a;
}

TAlarmState *alarmStateListAdd(
	TFlightObjectOther *f, EAlarmLevel l, int timeToEncounterSec)
{
	int newPrio = ALARM_STATE_PRIO(l, timeToEncounterSec);

	// Can we re-use an alarm state for this aircraft?
	for (int i = 0; i < sAlarmStateSize; i++) {
		TAlarmState *a = &sAlarmStates[i];
		if (a->flightObject == f) {
			if (newPrio > ALARM_STATE_PRIO(a->level, a->timeToEncounterSec)) {
				a->level = l;
				a->timeToEncounterSec = timeToEncounterSec;
				ocapLogStrIntInt("ALARM-UPD", a->level, a->timeToEncounterSec);
				alarmStateListSort();
			}
			return a;
		}
	}

	TAlarmState *aNew = NULL;
	if (sAlarmStateSize < ALARM_STATE_LIST_LENGTH) {
		// The list has some remaining space. Take the next free element.
		aNew = &sAlarmStates[sAlarmStateSize++];
	} else {
		// Select lowest prio entry via sAlarmStateOrder for replacement.
		int o = sAlarmStateOrder[sAlarmStateSize - 1];
		aNew = &sAlarmStates[o];
		// Don't insert the new alarm state if the list is full and has a lower prio than all existing items.
		int aLowestPrio = ALARM_STATE_PRIO(aNew->level, aNew->timeToEncounterSec);
		if (aLowestPrio > newPrio) {
			return NULL;
		}
	}

	aNew->flightObject = f;
	aNew->level = l;
	aNew->timeToEncounterSec = timeToEncounterSec;

	alarmStateListSort();
	return aNew;
}

static void alarmStateListSort(void)
{
	// Selection sort.
	// Start with unsorted array.
	for (int i = 0; i < sAlarmStateSize; i++) {
		sAlarmStateOrder[i] = i;
	}
	// Sort by moving the entry with the highest prio to the front.
	for (int i = 0; i < sAlarmStateSize; i++) {
		TAlarmState *aI = &sAlarmStates[sAlarmStateOrder[i]];
		int prioI = ALARM_STATE_PRIO(aI->level, aI->timeToEncounterSec);

		for (int j = i + 1; j < sAlarmStateSize; j++) {
			TAlarmState *aJ = &sAlarmStates[sAlarmStateOrder[j]];
			int prioJ = ALARM_STATE_PRIO(aJ->level, aJ->timeToEncounterSec);

			if (prioI < prioJ) {
				// Need to swap.
				int tmp = sAlarmStateOrder[i];
				sAlarmStateOrder[i] = sAlarmStateOrder[j];
				sAlarmStateOrder[j] = tmp;
				// One assignment is enough for the pointers as we don't need aJ any more.
				aI = aJ;
				prioI = prioJ;
			}
		}
	}
}

#if OCAP_SIMULATION == 1
#include <stdio.h>
void alarmStateListDump(void)
{
	printf(" * ");
	for (int i = 0, imax = alarmStateListGetCount(); i < imax; i++) {
		int o = sAlarmStateOrder[i];
		TAlarmState *a = alarmStateListGetAtIndex(i);
		printf(" -> [%d:%d,%d]", o, (int)a->level, a->timeToEncounterSec);
	}
	printf(" ->|\n");
}
#endif
