#include "OcapLog.h"
#include "AlarmService.h"


#define FAR_AWAY_METERS 1000000
#define ALARM_SERVICE_FACTOR 4


// For the moment, we maintain only 1 entry, for the most critical alarm.
// A future extension would be to maintain multiple entries and provide
// the most critical one to the client.
static TAlarmServiceEntry sEntry;


static void alarmServiceUpdate(TAlarmServiceEntry *e, int targetLevel, int distMtr);


void alarmServiceUpdateMostCritical(TAlarmState *a, int distMtr)
{
	// No alarm state: Decay slowly to level 0.
	if (!a) {
		alarmServiceUpdate(&sEntry, 0, -1);
		return;
	}
	// Convert alarm state levels (1 to 3) to our range (1 to 3*ALARM_SERVICE_FACTOR).
	int targetLevel = (int)a->level * ALARM_SERVICE_FACTOR;
	// On switching aircraft, boost to highest level right away, but decay slowly.
	if (sEntry.alarmState != a) {
		sEntry.alarmState = a;
		sEntry.distMtr = FAR_AWAY_METERS;
		sEntry.approaching = 1;
		alarmServiceUpdate(&sEntry, targetLevel, distMtr);
		return;
	}
	// Previously known alarm state: Quickly increase, slowly decrease.
	alarmServiceUpdate(&sEntry, targetLevel, distMtr);
}

TAlarmServiceEntry *alarmServiceGetMostCritical(void)
{
	return &sEntry;
}

EAlarmLevel alarmServiceGetLevel(TAlarmServiceEntry *e)
{
	if (!e || !e->alarmState || e->level <= 0) {
		return ALARM_LEVEL_NONE;
	}
	// No alarm if the distance between us and the aircraft is increasing.
	if (!e->approaching) {
		return ALARM_LEVEL_NONE;
	}
	// Rounds towards the higher alarm level.
	return (EAlarmLevel)((e->level + ALARM_SERVICE_FACTOR - 1) / ALARM_SERVICE_FACTOR);
}


static void alarmServiceUpdate(
	TAlarmServiceEntry *e, int targetLevel, int distMtr)
{
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
	// Drop after complete decay.
	if (e->level <= 0) {
		e->alarmState = NULL;
		e->level = 0;
	}
}
