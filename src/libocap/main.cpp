#include <iostream>
#include "AlarmStateList.h"
#include "FlightObjectList.h"
#include "FlightObjectOrientation.h"
#include "FlightPathExtrapolation.h"
#include "Prediction.h"
#include "CalculateOwnData.h"
#include "CalculateOtherData.h"


static void testAlarmStateList(void);
static void testFlightPathExtrapolation2rv(void);
static void testFlightPathExtrapolationRvz(void);
static void testPrediction(void);


int main(int argc, char *argv[])
{
	std::cout << "libocap-test" << std::endl;

	testAlarmStateList();

	testFlightPathExtrapolation2rv();
	testFlightPathExtrapolationRvz();

	testPrediction();

	return 0;
}

static void testAlarmStateList(void)
{
	alarmStateListClear();

	TFlightObjectOther *f1 = flightObjectListAddOther(0x1);
	alarmStateListAdd(f1, ALARM_LEVEL_2, 15);
	alarmStateListDump();

	TFlightObjectOther *f2 = flightObjectListAddOther(0x2);
	alarmStateListAdd(f2, ALARM_LEVEL_3, 7);
	alarmStateListDump();

	TFlightObjectOther *f3 = flightObjectListAddOther(0x3);
	alarmStateListAdd(f3, ALARM_LEVEL_1, 18);
	alarmStateListDump();

	TFlightObjectOther *f4 = flightObjectListAddOther(0x4);
	alarmStateListAdd(f4, ALARM_LEVEL_3, 5);
	alarmStateListDump();

	TFlightObjectOther *f5 = flightObjectListAddOther(0x5);
	alarmStateListAdd(f5, ALARM_LEVEL_1, 11);
	alarmStateListDump();

	TFlightObjectOther *f6 = flightObjectListAddOther(0x6);
	alarmStateListAdd(f6, ALARM_LEVEL_2, 9);
	alarmStateListDump();

	flightObjectListInit(0);
}

static void testFlightPathExtrapolation2rv(void)
{
	// Corresponds to test case 8 of the algorithm design test set.
	// (Start with v=0.6m/s in direction N, curve towards E with 0.2m/s2.)

	TVector r0_vec = { 0.0, 0.0, 0.0 };
	TVector v0_vec = { 0.0, 0.6, 0.0 };

	TVector r1_vec = { 0.09908, 0.58895, 0.0 };
	TVector v1_vec = { 0.19632, 0.56697, 0.0};

	TFlightPathExtrapolationData fpe;

	vectorCopy(&fpe.r0_vec, &r0_vec);
	vectorCopy(&fpe.v0_vec, &v0_vec);
	vectorCopy(&fpe.ri_vec, &r1_vec);
	vectorCopy(&fpe.vi_vec, &v1_vec);

	// Initialization step.

	flightPathExtrapolationPrepare2rv(&fpe);

	printf("    Z  = (%8.4f,%8.4f,%8.4f)\n", fpe.z_vec.x, fpe.z_vec.y, fpe.z_vec.z);
	printf("    r  = %8.4f\n", fpe.r);
	printf("    v  = %8.4f\n", fpe.v);
	printf("    theta    = %8.4f\n", fpe.theta);
	printf("    t_div_r  = %8.4f\n", fpe.t_div_r);

	// Extrapolation sequence

	for (int i = 0; i < 5; i++) {

		flightPathExtrapolationExecute(&fpe);

		printf("    r(%d)  = (%8.4f,%8.4f,%8.4f)\n", i, fpe.ri_vec.x, fpe.ri_vec.y, fpe.ri_vec.z);
	}
}

static void testFlightPathExtrapolationRvz(void)
{
	// Corresponds to test case 8 of the algorithm design test set.
	// (Start with v=0.6m/s in direction N, curve towards E with 0.2m/s2.)

	TVector r0_vec = { 0.0, 0.0, 0.0 };
	TVector v0_vec = { 0.0, 0.6, 0.0 };
	TVector z_vec =  { 1.8, 0.0, 0.0 };

	TFlightPathExtrapolationData fpe;

	vectorCopy(&fpe.r0_vec, &r0_vec);
	vectorCopy(&fpe.v0_vec, &v0_vec);
	vectorCopy(&fpe.ri_vec, &r0_vec);
	vectorCopy(&fpe.vi_vec, &v0_vec);
	vectorCopy(&fpe.z_vec, &z_vec);

	// Initialization step.

	flightPathExtrapolationPrepareRvz(&fpe);

	printf("    Z  = (%8.4f,%8.4f,%8.4f)\n", fpe.z_vec.x, fpe.z_vec.y, fpe.z_vec.z);
	printf("    r  = %8.4f\n", fpe.r);
	printf("    v  = %8.4f\n", fpe.v);
	printf("    theta    = %8.4f\n", fpe.theta);
	printf("    t_div_r  = %8.4f\n", fpe.t_div_r);

	// Extrapolation sequence

	for (int i = 0; i < 6; i++) {

		flightPathExtrapolationExecute(&fpe);

		printf("    r(%d)  = (%8.4f,%8.4f,%8.4f)\n", i, fpe.ri_vec.x, fpe.ri_vec.y, fpe.ri_vec.z);
	}
}

static void testPrediction(void)
{
	printf("testPredictions\n");

	uint32_t ts = 0;
	EOcapPathModel pathModel = OCAP_PATH_MODEL_LINEAR;

	// Create 2 flight objects (ours and another one).

	flightObjectListInit(1);

	TFlightObjectOwn *ownFlightObject = flightObjectListGetOwn();
	TFlightObjectOther *otherFlightObject = flightObjectListAddOther(2);
	TFlightObjectOther *thirdFlightObject = flightObjectListAddOther(3);

	// Provide data for the 2 flight objects.

	// In the first step, provide data only for our own airplane.
	ts = 0;
	TVector ownR0;
	TVector ownV0;
	vectorInit(&ownR0, 0, 0, 0);
	vectorInit(&ownV0, 0, 100, 0);
	calculateOwnDataFromGpsInfo(ts, &ownR0, &ownV0);

	// In the second step, provide data for our own and for two other airplanes.
	ts = 1;
	TVector ownR1;
	TVector ownV1;
	vectorInit(&ownR1, .99997, 99.99333, 0);
	// Flying from S to NNE, at (873,2823,0) after 30s
	vectorInit(&ownV1, 1.99987, 99.98000, 0);
	calculateOwnDataFromGpsInfo(ts, &ownR1, &ownV1);

	TVector otherR1;
	TVector otherV1;
	TVector otherZ;
	// Flying from W to E, at (873,2823,0) after 30s
	vectorInit(&otherR1, 0, 2823, 0);
	vectorInit(&otherV1, 30, 0, 0);
	vectorInit(&otherZ, 0, -50000, 0);
	calculateOtherDataFromInfo(otherFlightObject, ts, &otherR1, &otherV1, &otherZ, pathModel);

	TVector thirdR1;
	TVector thirdV1;
	TVector thirdZ;
	// Flying from W to E, at (873,2823,0) after 30s
	vectorInit(&thirdR1, 1400, 3400, 0);
	vectorInit(&thirdV1, 3, 3, 0);
	vectorInit(&thirdZ, 0, +40000, 0);
	calculateOtherDataFromInfo(thirdFlightObject, ts, &thirdR1, &thirdV1, &thirdZ, pathModel);

	// Configure collision prediction.

	predictionInit(30, 2, 4);

	// Perform collision prediction.

	predictionCalculateAlarmStates(ts);

	// Show the result.

	alarmStateListDump();

	TFlightObjectOrientation fo;

	int hasOrientation = flightObjectOrientationCalculate(&fo, ownFlightObject, otherFlightObject);
	printf("other: hasOrientation = %d  (angle=%d deg, dist=%d mtr)\n", hasOrientation, fo.directionDeg, fo.distanceMeters);

	hasOrientation = flightObjectOrientationCalculate(&fo, ownFlightObject, thirdFlightObject);
	printf("third: hasOrientation = %d  (angle=%d deg, dist=%d mtr)\n", hasOrientation, fo.directionDeg, fo.distanceMeters);
}
