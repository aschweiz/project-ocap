//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L Example and test code.
// (Based on EASA ADS-L 4 SRD860 Issue 1)
//
// 10.09.2024 ASR  First version.
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

#include <iostream>
#include <iomanip>

#include "create_ads_l_packet.h"
#include "ads_l_encode_iconspicuity.h"

#include "ads_l_decode_iconspicuity.h"
#include "decode_ads_l_packet.h"

int gNofFailedAssertions;

static void testIConspicuity();
static void testIConspicuityV2();

static void testEncoding(uint8_t *dataOut22);
static void testDecoding(uint8_t *dataIn22);

static void testEncodingV2(uint8_t *dataOut26);
static void testDecodingV2(uint8_t *dataIn26);

static void showBytes(uint8_t *bytes, int len);

static void assertValue(std::string msg, double vExpected, double vActual, double tolerancePercent);
static void assertValue(std::string msg, int vExpected, int vActual);
static void assertBytes(std::string msg, uint8_t *bytesExpected, uint8_t *bytesActual, int len);
static void showTestResult();

int main(int argc, char *argv[])
{
	std::cout << "libadsl-test" << std::endl;

	testIConspicuity();
	testIConspicuityV2();

	showTestResult();
	return 0;
}

static void testIConspicuity()
{
	uint8_t adslData22[22];

	testEncoding(&adslData22[0]);
	testDecoding(&adslData22[0]);
}

static void testIConspicuityV2()
{
	uint8_t adslData26[26];

	testEncodingV2(&adslData26[0]);
	testDecodingV2(&adslData26[0]);
}

static void testEncoding(uint8_t *dataOut22)
{
	SGpsData gpsData;
	gpsData.ts_sec_in_hour = 200;
	gpsData.lat_deg_e7 = 475000000; // 47.5N
	gpsData.lon_deg_e7 =  85000000; // 8.5E
	gpsData.height_m = 583;
	gpsData.hacc_cm = 170;
	gpsData.vacc_cm = 440;
//	gpsData.vel_n_cm_s = 350; // not used
//	gpsData.vel_e_cm_s = 140; // not used
	gpsData.vel_u_cm_s = -80;
	gpsData.gspeed_cm_s = 220;
	gpsData.heading_deg_e1 = 400; // 40 deg
	gpsData.sacc_cm_s = 200;
//	gpsData.cacc_deg_e1 = 10; // +/- 1 deg // not used

	SAircraftConfig aircraftConfig;
	aircraftConfig.addrMapEntry = 9; // Manuf. page 0
	aircraftConfig.addr = 0x123456;
	aircraftConfig.flightState = ADSL_ICONSP_FLIGHT_STATE_AIRBORNE;
	aircraftConfig.acftCategory = ADSL_ICONSP_AIRCRAFT_CATEGORY_ROTORCRAFT;

	// From GPS and aircraft state to ADS-L iConspicuity packet.
	SAdslIConspicuity adslPacket;
	createAdslPacket(&gpsData, &aircraftConfig, &adslPacket);

	// From ADS-L iConspicuity packet to data bytes for transfer.
	adslEncodeIConspicuity(&adslPacket, dataOut22, 22);

	showBytes(dataOut22, 22);

	uint8_t expectedBytes[] = {
		0x18, 0x00, 0x02, 0x24, 0x48, 0xd1, 0x58, 0x52,
		0x18, 0x43, 0x8e, 0x15, 0x06, 0x0b, 0x5d, 0x08,
		0x0e, 0x1e, 0x0c, 0x38, 0x00, 0xfc
	};
	assertBytes("Packet data", &expectedBytes[0], dataOut22, 22);
}

static void testDecoding(uint8_t *dataIn22)
{
	// From data bytes to ADS-L iConspicuity packet.
	SAdslIConspicuity adslPacket;
	EAdslDecodeResult result = adslDecodeIConspicuity(dataIn22, &adslPacket, 1);

	SGpsData gpsData;
	SAircraftConfig aircraftConfig;
	decodeAdslPacket(&adslPacket, &gpsData, &aircraftConfig);

	assertValue("Lat", 475000000, gpsData.lat_deg_e7, 1);
	assertValue("Lon",  85000000, gpsData.lon_deg_e7, 1);
	assertValue("Height",    583, gpsData.height_m, 2);
	assertValue("Vel_u",     -80, gpsData.vel_u_cm_s, 10);
	assertValue("Gspeed",    220, gpsData.gspeed_cm_s, 10);
	assertValue("Heading",   400, gpsData.heading_deg_e1, 10);

	assertValue("hacc",      300, gpsData.hacc_cm);		// 170 -> 300
	assertValue("vacc",     1500, gpsData.vacc_cm);		// 440 -> 1500
	assertValue("sacc",      300, gpsData.sacc_cm_s);	// 200 -> 300

	assertValue("ID",   0x123456, aircraftConfig.addr);
	assertValue("ID map",      9, aircraftConfig.addrMapEntry);
	assertValue("Acft",  (int)ADSL_ICONSP_AIRCRAFT_CATEGORY_ROTORCRAFT, aircraftConfig.acftCategory);
	assertValue("State", (int)ADSL_ICONSP_FLIGHT_STATE_AIRBORNE, aircraftConfig.flightState);
}


static void testEncodingV2(uint8_t *dataOut26)
{
	SGpsData gpsData;
	gpsData.ts_sec_in_hour = 200;
	gpsData.lat_deg_e7 = 475000000; // 47.5N
	gpsData.lon_deg_e7 =  85000000; // 8.5E
	gpsData.height_m = 583;
	gpsData.hacc_cm = 170;
	gpsData.vacc_cm = 440;
//	gpsData.vel_n_cm_s = 350; // not used
//	gpsData.vel_e_cm_s = 140; // not used
	gpsData.vel_u_cm_s = -80;
	gpsData.gspeed_cm_s = 220;
	gpsData.heading_deg_e1 = 400; // 40 deg
	gpsData.sacc_cm_s = 200;
//	gpsData.cacc_deg_e1 = 10; // +/- 1 deg // not used

	SAircraftConfig aircraftConfig;
	aircraftConfig.addrMapEntry = 9; // Manuf. page 0
	aircraftConfig.addr = 0x123456;
	aircraftConfig.flightState = ADSL_ICONSP_FLIGHT_STATE_AIRBORNE;
	aircraftConfig.acftCategory = ADSL_ICONSP_AIRCRAFT_CATEGORY_ROTORCRAFT;

	// From GPS and aircraft state to ADS-L iConspicuity2 packet.
	SAdslIConspicuity2 adslPacket;
	int z[3] = { 14, -333, 1234 };
	EAdslIConspicuity2PathModel pm = ADSL_ICONSP2_PATH_MODEL_ARC;
	createAdslPacket2(&gpsData, &aircraftConfig, &z[0], pm, &adslPacket);

	// From ADS-L iConspicuity2 packet to data bytes for transfer.
	adslEncodeIConspicuity2(&adslPacket, dataOut26, 26);

	showBytes(dataOut26, 26);

	uint8_t expectedBytes[] = {
		0x1c, 0x00, 0x03, 0x24, 0x48, 0xd1, 0x58, 0x52,
		0x18, 0x43, 0x8e, 0x15, 0x06, 0x0b, 0x5d, 0x08,
		0x0e, 0x1e, 0x0c, 0x38, 0x00, 0xfc,
		// Path model and Z offset vector:
		0x80, 0xea, 0x8d, 0x11
	};
	assertBytes("Packet data", &expectedBytes[0], dataOut26, 26);
}

static void testDecodingV2(uint8_t *dataIn26)
{
	// From data bytes to ADS-L iConspicuity packet.
	SAdslIConspicuity2 adslPacket;
	EAdslDecodeResult result = adslDecodeIConspicuity2(dataIn26, &adslPacket);

	SGpsData gpsData;
	SAircraftConfig aircraftConfig;
	int z[3] = { 0 };
	EAdslIConspicuity2PathModel pm;
	decodeAdslPacket2(&adslPacket, &gpsData, &aircraftConfig, &z[0], &pm);

	assertValue("Lat", 475000000, gpsData.lat_deg_e7, 1);
	assertValue("Lon",  85000000, gpsData.lon_deg_e7, 1);
	assertValue("Height",    583, gpsData.height_m, 2);
	assertValue("Vel_u",     -80, gpsData.vel_u_cm_s, 10);
	assertValue("Gspeed",    220, gpsData.gspeed_cm_s, 10);
	assertValue("Heading",   400, gpsData.heading_deg_e1, 10);

	assertValue("hacc",      300, gpsData.hacc_cm);		// 170 -> 300
	assertValue("vacc",     1500, gpsData.vacc_cm);		// 440 -> 1500
	assertValue("sacc",      300, gpsData.sacc_cm_s);	// 200 -> 300

	assertValue("ID",   0x123456, aircraftConfig.addr);
	assertValue("ID map",      9, aircraftConfig.addrMapEntry);
	assertValue("Acft",  (int)ADSL_ICONSP_AIRCRAFT_CATEGORY_ROTORCRAFT, aircraftConfig.acftCategory);
	assertValue("State", (int)ADSL_ICONSP_FLIGHT_STATE_AIRBORNE, aircraftConfig.flightState);

	assertValue("z[0]",       14, z[0]);	//   14 ->   14
	assertValue("z[1]",     -332, z[1]);	// -333 -> -332
	assertValue("z[2]",     1232, z[2]);	// 1234 -> 1232

	assertValue("PathModel",   2, (int)pm); // ARC = 2
}


static void showBytes(uint8_t *bytes, int len)
{
	// Display the individual bytes in the packet.
	std::cout << "Packet bytes: " << std::endl;
	std::ios::fmtflags f(std::cout.flags());
	std::cout << std::hex;
	for (int i = 0; i < len; i++) {
		std::cout << std::setfill('0') << std::setw(2) << (int)bytes[i] << " ";
	}
	std::cout << std::endl;
	std::cout.flags(f);
}

static void assertValue(std::string msg, double vExpected, double vActual, double tolerancePercent)
{
	if (vExpected < 0) {
		tolerancePercent = -tolerancePercent;
	}
	double vmin = vExpected * (100.0 - tolerancePercent) * 0.01;
	double vmax = vExpected * (100.0 + tolerancePercent) * 0.01;
	if (vmin <= vActual && vActual <= vmax) {
		return;
	}
	std::cout << msg << " " << vActual << std::endl;
	gNofFailedAssertions++;
}

static void assertValue(std::string msg, int vExpected, int vActual)
{
	if (vExpected == vActual) {
		return;
	}
	std::cout << msg << " " << vActual << std::endl;
	gNofFailedAssertions++;
}

static void assertBytes(std::string msg, uint8_t *bytesExpected, uint8_t *bytesActual, int len)
{
	bool allOk = true;
	for (int i = 0; i < len; i++) {
		if (bytesActual[i] == bytesExpected[i]) {
			continue;
		}
		allOk = false;
		std::cout
			<< msg << " "
			<< " Index: " << i
			<< ", Expected: " << std::hex << (int)bytesExpected[i]
			<< ", Actual: " << std::hex << (int)bytesActual[i]
			<< std::endl;
	}
	if (!allOk) {
		gNofFailedAssertions++;
	}
}

static void showTestResult()
{
	if (gNofFailedAssertions == 0) {
		std::cout << "SUCCESS! All tests passed." << std::endl;
		return;
	}

	std::cout << "FAILURE! " << gNofFailedAssertions << " test(s) failed." << std::endl;
	exit(-1);
}
