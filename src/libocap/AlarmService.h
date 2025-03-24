//
// AlarmService.h
//
// Alarm state filtering.
//

#ifndef __ALARM_SERVICE_H__
#define __ALARM_SERVICE_H__ 1

#include "AlarmState.h"

typedef struct {
	TAlarmState *alarmState;
	int level;
	int distMtr;
	int approaching;
} TAlarmServiceEntry;

// If the distance is not available, pass -1 to this function.
void alarmServiceUpdateMostCritical(TAlarmState *a, int distMtr);

TAlarmServiceEntry *alarmServiceGetMostCritical(void);

EAlarmLevel alarmServiceGetLevel(TAlarmServiceEntry *e);

#endif // __ALARM_SERVICE_H__
