//
// OCAP - Open Collision Avoidance Protocol
//
// ADS-L Example and test code.
// (Based on EASA ADS-L 4 SRD860 Issue 1)
//
// 10.09.2024 ASR  First version.
// 14.05.2025 ASR  Merged OCAP extension into IConspicuity packet structure.
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

#include "ads_l_xxtea.h"

#include "create_ads_l_packet.h"
#include "ads_l_encode_iconspicuity.h"

#include "ads_l_decode_iconspicuity.h"
#include "decode_ads_l_packet.h"

int gNofFailedAssertions;

static void testXxteaCompatibility();

static void testIConspicuityCompatibility();

static void testIConspicuity();
static void testIConspicuityV2();

static void testEncoding(uint8_t *dataOut22);
static void testDecoding(uint8_t *dataIn22);

static void testEncodingV2(uint8_t *dataOut30);
static void testDecodingV2(uint8_t *dataIn30);

static void showBytes(uint8_t *bytes, int len);

static void assertValue(std::string msg, double vExpected, double vActual, double tolerancePercent);
static void assertValue(std::string msg, int vExpected, int vActual);
static void assertBytes(std::string msg, uint8_t *bytesExpected, uint8_t *bytesActual, int len);
static void showTestResult();

int main(int argc, char *argv[])
{
	std::cout << "libadsl-test" << std::endl;

	testXxteaCompatibility();

	testIConspicuityCompatibility();

	testIConspicuity();
	testIConspicuityV2();

	showTestResult();
	return 0;
}

static void testXxteaCompatibility()
{
	uint32_t data[] = {
		0xc2e38f4b,
		0x008bc012,
		0xb9aa56ad,
		0x10ba7119,
		0x7d9c9a8e
	};

	// Decode

	adslXxteaDecodeWithPubkey(data, 5);

	uint32_t dataExpected1[] = {
		0x5675c802,
		0x85071004,
		0x06764354,
		0x03b40006,
		0x5e005900
	};

	for (int i = 0; i < 5; i++ ) {
		assertValue("xxtea data decode", (int)dataExpected1[i], (int)data[i]);
	}

	// Encode

	adslXxteaEncodeWithPubkey(data, 5);

	uint32_t dataExpected2[] = {
		0xc2e38f4b,
		0x008bc012,
		0xb9aa56ad,
		0x10ba7119,
		0x7d9c9a8e
	};

	for (int i = 0; i < 5; i++ ) {
		assertValue("xxtea data encode", (int)dataExpected2[i], (int)data[i]);
	}
}

static void testIConspicuityCompatibility()
{
	// Test decoding and re-encoding a recorded packet from a Skytraxx device.
	uint8_t dataIn25[25] = {
		0x18, 0x00, 0x02, 0xc8, 0x75, 0x56, 0x04, 0x2c,
		0x07, 0x87, 0x54, 0x43, 0x6e, 0x06, 0x06, 0x00,
		0xaf, 0x03, 0x00, 0x97, 0x00, 0x5f, 0x0a, 0x35,
		0x1e
	};

	// From data bytes to ADS-L iConspicuity packet.
	SAdslIConspicuity adslPacket;
	EAdslDecodeResult result = adslDecodeIConspicuity(dataIn25, 25, &adslPacket);
	assertValue("testIConspicuityCompatibility decode result", ADSL_DECODE_SUCCESS, result);

	SGpsData gpsData;
	SAircraftConfig aircraftConfig;
	EAdslIConspicuityOcapPathModel pathModel;
	int z[3] = { 0 };
	decodeAdslPacket(&adslPacket, &gpsData, &aircraftConfig, &pathModel, &z[0]);

	assertValue("Lat", 473419200, gpsData.lat_deg_e7, 1);
	assertValue("Lon",  84728880, gpsData.lon_deg_e7, 1);
	assertValue("Height",    623, gpsData.height_m, 2);
	assertValue("Vel_u",       0, gpsData.vel_u_cm_s, 10);
	assertValue("Gspeed",      0, gpsData.gspeed_cm_s, 10);
	assertValue("Heading",  2123, gpsData.heading_deg_e1, 10);

	assertValue("hacc",      300, gpsData.hacc_cm);
	assertValue("vacc",     1500, gpsData.vacc_cm);
	assertValue("sacc",      300, gpsData.sacc_cm_s);

	assertValue("ID",   0x1159d7, aircraftConfig.addr);
	assertValue("ID map",      8, aircraftConfig.addrMapEntry);
	assertValue("Acft",  (int)ADSL_ICONSP_AIRCRAFT_CATEGORY_HANG_PARA_GLIDER, aircraftConfig.acftCategory);
	assertValue("State", (int)ADSL_ICONSP_FLIGHT_STATE_UNDEF, aircraftConfig.flightState);

	// From GPS and aircraft state to ADS-L iConspicuity packet.
	SAdslIConspicuity adslPacketOut;
	int zOut[3] = { 0 };
	createAdslPacket(
		&gpsData, &aircraftConfig, &adslPacketOut,
		ADSL_ICONSP2_PATH_MODEL_LINEAR, zOut);

	// From ADS-L iConspicuity packet to data bytes for transfer.
	uint8_t dataOut25[25];
	int encodeResult = adslEncodeIConspicuity(&adslPacket, dataOut25, 25);
	assertValue("testIConspicuityCompatibility encode result", 0, encodeResult);

	showBytes(dataOut25, 22);

	assertBytes("Packet data", &dataIn25[0], dataOut25, 22);
}


static void testIConspicuity()
{
	uint8_t adslData22[22] = { 0 };

	testEncoding(&adslData22[0]);
	testDecoding(&adslData22[0]);
}

static void testIConspicuityV2()
{
	uint8_t adslData30[30] = { 0 };

	testEncodingV2(&adslData30[0]);
	testDecodingV2(&adslData30[0]);
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
	int z[3] = { 0 };
	createAdslPacket(
		&gpsData, &aircraftConfig, &adslPacket,
		ADSL_ICONSP2_PATH_MODEL_LINEAR, z);

	// From ADS-L iConspicuity packet to data bytes for transfer.
	int encodeResult = adslEncodeIConspicuity(&adslPacket, dataOut22, 22);
	assertValue("testEncoding encode result", 0, encodeResult);

	showBytes(dataOut22, 22);

	uint8_t expectedBytes[] = {
		0x18, 0x00, 0x02, 0x89, 0x15, 0x8d, 0x04, 0x94,
		0x03, 0x15, 0x8e, 0x43, 0x5d, 0x0b, 0x06, 0x08,
		0x87, 0x83, 0x41, 0x1c, 0x00, 0x5f
	};
	assertBytes("Packet data", &expectedBytes[0], dataOut22, 22);
}

static void testDecoding(uint8_t *dataIn22)
{
	// From data bytes to ADS-L iConspicuity packet.
	SAdslIConspicuity adslPacket;
	EAdslDecodeResult result = adslDecodeIConspicuity(
		dataIn22, 22, &adslPacket);
	assertValue("testIConspicuityCompatibility decode result", ADSL_DECODE_SUCCESS, result);

	SGpsData gpsData;
	SAircraftConfig aircraftConfig;
	EAdslIConspicuityOcapPathModel pathModel;
	int z[3] = { 0 };
	decodeAdslPacket(&adslPacket, &gpsData, &aircraftConfig, &pathModel, &z[0]);

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


static void testEncodingV2(uint8_t *dataOut30)
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

	// From GPS and aircraft state to ADS-L iConspicuity packet with OCAP extension.
	SAdslIConspicuity adslPacket;
	int z[3] = { 14, -333, 1234 };
	EAdslIConspicuityOcapPathModel pm = ADSL_ICONSP2_PATH_MODEL_ARC;
	createAdslPacket(&gpsData, &aircraftConfig, &adslPacket, pm, &z[0]);

	// From ADS-L iConspicuity packet to data bytes for transfer.
	int encodeResult = adslEncodeIConspicuity(&adslPacket, dataOut30, 30);
	assertValue("testEncodingV2 encode result", 0, encodeResult);

	showBytes(dataOut30, 30);

	uint8_t expectedBytes[] = {
		0x18, 0x00, 0x02, 0x89, 0x15, 0x8d, 0x04, 0x94,
		0x03, 0x15, 0x8e, 0x43, 0x5d, 0x0b, 0x06, 0x08,
		0x87, 0x83, 0x41, 0x1c, 0x00, 0xdf,
		// checksum
		0x00, 0x00, 0x00,
		// Path model and Z offset vector:
		0x80, 0xea, 0x8d, 0x11,
		// checksum
		0xf6
	};
	assertBytes("Packet data", &expectedBytes[0], dataOut30, 30);
}

static void testDecodingV2(uint8_t *dataIn30)
{
	// From data bytes to ADS-L iConspicuity packet.
	SAdslIConspicuity adslPacket;
	EAdslDecodeResult result = adslDecodeIConspicuity(dataIn30, 30, &adslPacket);
	assertValue("testIConspicuityCompatibility decode result", ADSL_DECODE_SUCCESS, result);

	SGpsData gpsData;
	SAircraftConfig aircraftConfig;
	int z[3] = { 0 };
	EAdslIConspicuityOcapPathModel pm;
	decodeAdslPacket(&adslPacket, &gpsData, &aircraftConfig, &pm, &z[0]);

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
