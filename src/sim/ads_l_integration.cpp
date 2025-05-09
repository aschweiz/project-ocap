//
// ads_l_integration.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Integrating with the ADS-L library.
//
// 12.09.2024 ASR  First version
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

#include "ads_l_integration.h"
#include "ads_l_decode_iconspicuity.h"
#include "ads_l_encode_iconspicuity.h"
#include "aircraft_config.h"
#include "create_ads_l_packet.h"
#include "decode_ads_l_packet.h"
#include "gps_data.h"
#include "lon_lat_util.h"

using namespace linalg;

bool CreateRadioMessage(
    RadioMessage *msg, int addr,
    long txMs, Vector3d &posNewMtr, Vector3d &velNewMtrSec,
    Vector3d *zVec, EOcapPathModel pathModel)
{
    TVector posNewDeg;
    lonLatAltMtrToDegDegMtr(posNewMtr.GetVector(), &posNewDeg);

    int32_t lonDegE_E5 = (int32_t)(posNewDeg.x * 1e5);
    int32_t latDegN_E5 = (int32_t)(posNewDeg.y * 1e5);
    int32_t altMtr = (int32_t)posNewDeg.z;

    int32_t zVelCmPerS = velNewMtrSec.Z() * 100; // m/s --> cm/s
    double lonVelCmPerS = velNewMtrSec.X() * 100; // m/s --> cm/s
    double latVelCmPerS = velNewMtrSec.Y() * 100; // m/s --> cm/s

    int32_t gspeedCmPerS = (int32_t)sqrt(lonVelCmPerS * lonVelCmPerS + latVelCmPerS * latVelCmPerS);

    double headingAtanDeg = -atan2(latVelCmPerS, lonVelCmPerS) * 180 / M_PI + 90;
    int32_t headingDeg_E1 = (int32_t)(headingAtanDeg * 10);
    if (headingDeg_E1 > 1800) {
        headingDeg_E1 = headingDeg_E1 - 3600;
    }

    SGpsData gpsData;
    gpsData.ts_sec_in_hour = (txMs / 1000) % 3600; // TODO would need hh:mm:ss format
    gpsData.lat_deg_e7 = latDegN_E5 * 100; // 47.5N
    gpsData.lon_deg_e7 = lonDegE_E5 * 100; // 8.5E
    gpsData.height_m = altMtr;
    gpsData.hacc_cm = 200;
    gpsData.vacc_cm = 500;
    //	gpsData.vel_n_cm_s = 350; // not used
    //	gpsData.vel_e_cm_s = 140; // not used
    gpsData.vel_u_cm_s = zVelCmPerS;
    gpsData.gspeed_cm_s = gspeedCmPerS;
    gpsData.heading_deg_e1 = headingDeg_E1;
    gpsData.sacc_cm_s = 200;
    //	gpsData.cacc_deg_e1 = 10; // +/- 1 deg // not used

    SAircraftConfig aircraftConfig;
    aircraftConfig.addrMapEntry = 9; // Manuf. page 0
    aircraftConfig.addr = addr;
    aircraftConfig.flightState = ADSL_ICONSP_FLIGHT_STATE_AIRBORNE;
    aircraftConfig.acftCategory = ADSL_ICONSP_AIRCRAFT_CATEGORY_UNDEF;

    // From GPS and aircraft state to ADS-L iConspicuity packet.
    if (!zVec) {
        SAdslIConspicuity adslPacket;
        createAdslPacket(&gpsData, &aircraftConfig, &adslPacket);
        // From ADS-L iConspicuity packet to data bytes for transfer.
        adslEncodeIConspicuity(&adslPacket, &msg->bytes[0], MAX_RADIO_MESSAGE_SIZE_BYTES);

    } else {
        SAdslIConspicuity2 adslPacket;

        // This is an 1:1 mapping.
        EAdslIConspicuity2PathModel pm = (EAdslIConspicuity2PathModel)pathModel;

        // The Z vector is encoded as an offset from the current position
        // in .125*v units.
        double v = 0.01 * sqrt(gpsData.gspeed_cm_s * gpsData.gspeed_cm_s + gpsData.vel_u_cm_s * gpsData.vel_u_cm_s);
        int zV8[3] = {
            (int)(zVec->X() * 8 / v),
            (int)(zVec->Y() * 8 / v),
            (int)(zVec->Z() * 8 / v)
        };

        createAdslPacket2(&gpsData, &aircraftConfig, zV8, pm, &adslPacket);

        // From ADS-L iConspicuity packet to data bytes for transfer.
        adslEncodeIConspicuity2(&adslPacket, &msg->bytes[0], MAX_RADIO_MESSAGE_SIZE_BYTES);
    }

    // Other fields of the message.
    msg->txStartTimeMs = txMs; // s.200 ... s.799
    msg->txDurationMs = 5.0;

    return true;
}

bool DecodeRadioMessage(
    RadioMessage &msg,
    Vector3d &posMtr, Vector3d &velMtrSec,
    bool *hasZ, Vector3d &zVecMtr, EOcapPathModel *pathModel)
{
    SGpsData gpsData;
    SAircraftConfig aircraftConfig;

    *hasZ = false;

    // From data bytes to ADS-L iConspicuity or iConspicuity2 packet.
    if (msg.bytes[2] == ADSL_PAYLOAD_TYPE_BROADCAST_ICONSPICUITY) {
        SAdslIConspicuity adslPacket;
        EAdslDecodeResult result = adslDecodeIConspicuity(&msg.bytes[0], &adslPacket, 1);
        if (result != ADSL_DECODE_SUCCESS) {
            return false;
        }
        decodeAdslPacket(&adslPacket, &gpsData, &aircraftConfig);

    } else if (msg.bytes[2] == ADSL_PAYLOAD_TYPE_BROADCAST_ICONSPICUITY2) {
        SAdslIConspicuity2 adslPacket;
        EAdslDecodeResult result = adslDecodeIConspicuity2(&msg.bytes[0], &adslPacket);
        if (result != ADSL_DECODE_SUCCESS) {
            return false;
        }
        int zIntV8[3] = { 0 };
        EAdslIConspicuity2PathModel pm;
        decodeAdslPacket2(&adslPacket, &gpsData, &aircraftConfig, &zIntV8[0], &pm);
        *pathModel = (EOcapPathModel)pm;
        // The Z offset is stored in multiples of 0.125 * v.
        double v = 0.01 * sqrt(gpsData.gspeed_cm_s * gpsData.gspeed_cm_s + gpsData.vel_u_cm_s * gpsData.vel_u_cm_s);
        zVecMtr.Set(
            (double)zIntV8[0] * v / 8,
            (double)zIntV8[1] * v / 8,
            (double)zIntV8[2] * v / 8);
        *hasZ = true;
    }

    // x = lon, y = lat, z = alt
    Vector3d posDeg(
        (double)gpsData.lon_deg_e7 / 1e7,
        (double)gpsData.lat_deg_e7 / 1e7,
        (double)gpsData.height_m);

    lonDegLatDegAltMtrToMtrMtrMtr(posDeg.GetVector(), posMtr.GetVector());

    double headingRad = -(double)(gpsData.heading_deg_e1 - 900) / 1800.0f * M_PI;
    double gspeedMtrPerS = (double)gpsData.gspeed_cm_s / 100.0f;

    double sinfHeadingRad = sinf(headingRad);
    double cosfHeadingRad = cosf(headingRad);
    double lonMtrPerSec = gspeedMtrPerS * cosfHeadingRad;
    double latMtrPerSec = gspeedMtrPerS * sinfHeadingRad;

    // x = lon, y = lat, z = alt
    velMtrSec.Set(lonMtrPerSec, latMtrPerSec, (double)gpsData.vel_u_cm_s * 0.01);

    return true;
}

void FixPacketAltitude(RadioMessage &msg, int altMtr)
{
    // From data bytes to ADS-L iConspicuity packet.
    SAdslIConspicuity adslPacket;
    EAdslDecodeResult result = adslDecodeIConspicuity(&msg.bytes[0], &adslPacket, 1);

    SGpsData gpsData;
    SAircraftConfig aircraftConfig;
    decodeAdslPacket(&adslPacket, &gpsData, &aircraftConfig);

    gpsData.height_m = altMtr;

    // From GPS and aircraft state to ADS-L iConspicuity packet.
    SAdslIConspicuity adslPacketFixed;
    createAdslPacket(&gpsData, &aircraftConfig, &adslPacketFixed);

    // From ADS-L iConspicuity packet to data bytes for transfer.
    adslEncodeIConspicuity(&adslPacketFixed, &msg.bytes[0], MAX_RADIO_MESSAGE_SIZE_BYTES);
}
