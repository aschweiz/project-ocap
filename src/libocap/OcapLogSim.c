//
// OcapLogSim.c
//
// OCAP - Open Collision Avoidance Protocol
//
// Logging implementation for using the OCAP library in a simulator.
//
// 02.07.2024 ASR  First version.
//
// Software License (BSD):
// Copyright 2024 Classy Code GmbH.
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

#include <stdio.h>
#include "OcapLog.h"


void ocapLogStrInt(const char *str, int i)
{
	printf("OCAP,%s,%d\n", str, i);
}


void ocapLogStrIntInt(const char *str, int i, int j)
{
	printf("OCAP,%s,%d,%d\n", str, i, j);
}

void ocapLogFlVec(int own, TVector *pos, TVector *vel)
{
	// Print position and velocity vector.
	printf("OCAP,FL-%s-VEC,%d,%d,%d,%d,%d,%d\n", own ? "OWN" : "OTHER",
		(int)pos->x, (int)pos->y, (int)pos->z,
		(int)vel->x, (int)vel->y, (int)vel->z);
}

void ocapLogFlTN(int nr, TVector *vecT, TVector *vecN)
{
	printf("OCAP,FL-T-N,%d,%f,%f,%f,%f,%f,%f\n",
		nr,
		vecT->x, vecT->y, vecT->z,
		vecN->x, vecN->y, vecN->z);
}

void ocapLogFlOwn(TFlightObjectOwn *flOwn)
{
	int ix = FLIGHT_OBJECT_OWN_EXTRAPOLATION_DEPTH_SEC - 1;
	printf("OCAP,FL-OWN,%d,%d,%d,%d\n",
		(int)flOwn->pos_i[ix].x, (int)flOwn->pos_i[ix].y,
		(int)flOwn->rxPos.x, (int)flOwn->rxPos.y);
}

void ocapLogFlOwnPath(TVector *ownFlightPath)
{
	for (int i = 0; i < 4; i++) {
		printf("OCAP,FL-OWN-PATH,%d,%d,%d\n",
			i, (int)ownFlightPath[i].x, (int)ownFlightPath[i].y);
	}
}

void ocapLogFlOtherPath1(TVector *otherPos, int t)
{
	printf("OCAP,FL-OTHER-PATH,%d,%d,%d\n",
		t, (int)otherPos->x, (int)otherPos->y);

}

void ocapLogFlOtherTs(int rxTs, int ts)
{
	printf("OCAP,FL-OTHER-TS,%d,%d\n", rxTs, ts);
}

void ocapLogFpe(int own, TFlightPathExtrapolationData *fpe)
{
	printf("OCAP,FL-%s-FPE-1,%d,%d,%d,%d\n", own ? "OWN" : "OTHER",
		(int)fpe->r0_vec.x, (int)fpe->r0_vec.y,
		(int)fpe->ri_vec.x, (int)fpe->ri_vec.y);
}

void ocapLogRZxyV(int own, TFlightPathExtrapolationData *fpe)
{
	if (fpe->predictionModel == OCAP_PATH_MODEL_LINEAR) {
		printf("OCAP,FL-%s-R-Zxy-V,LINEAR\n", own ? "OWN" : "OTHER");
		return;
	}

	if (fpe->predictionModel == OCAP_PATH_MODEL_SPHERIC) {
		printf("OCAP,FL-%s-R-Zxy-V,SPHERIC,%d,%d,%d,%d\n", own ? "OWN" : "OTHER",
			(int)fpe->r, (int)fpe->z_vec.x, (int)fpe->z_vec.y, (int)fpe->v);
		return;
	}

	printf("OCAP,FL-%s-R-Zxy-V,ARC,%d,%d,%d,%d\n", own ? "OWN" : "OTHER",
		(int)fpe->r, (int)fpe->z_vec.x, (int)fpe->z_vec.y, (int)fpe->v);
}

void ocapLogModelZxyzV10(int own, EOcapPathModel pathModel, int zx, int zy, int zz, int v10)
{
	// FL-OWN-Zxyz-V8,-11,2,5,16
	if (pathModel == OCAP_PATH_MODEL_LINEAR) {
		printf("OCAP,FL-%s-Zxyz-V8-LINEAR,%d,%d,%d,%d\n", own ? "OWN" : "OTHER", zx, zy, zz, v10);
	} else if (pathModel == OCAP_PATH_MODEL_SPHERIC) {
		printf("OCAP,FL-%s-Zxyz-V8-SPHERIC,%d,%d,%d,%d\n", own ? "OWN" : "OTHER", zx, zy, zz, v10);
	} else if (pathModel == OCAP_PATH_MODEL_ARC) {
		printf("OCAP,FL-%s-Zxyz-V8-ARC,%d,%d,%d,%d\n", own ? "OWN" : "OTHER", zx, zy, zz, v10);
	} else {
		printf("OCAP,FL-%s-Zxyz-V8-?,%d,%d,%d,%d\n", own ? "OWN" : "OTHER", zx, zy, zz, v10);
	}
}
