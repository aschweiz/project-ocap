//
// main.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Driver program for the "flp" importer tool.
// This tool can create "flp" (flight path) files based on IGC- and CSV-files.
// The input files are sampled and smoothed with a spline function. To create
// the flp output file, we then sample the spline function in fixed intervals
// of 1 second.
//
// 09.11.2023 ASR  First version
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

#include <ctype.h>
#include <math.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "igc_parser.h"
#include "linalg.h"
#include "splines.h"

// Sampling interval for the IGC files.
// The flight path is reconstructed with spline functions and too dense
// sampling would cause artefacts in the output data.
#define SOURCE_INTERVAL_SEC 4

using namespace linalg;
using namespace splines;

static void createFlightPathFileFromIgc(IgcFile *igc, FILE *flp);
static void createFlightPathFileFromCsv(std::ifstream &csv, FILE *flp);
static void saveFlightPath(Spline_CentripetalCatmullRom &spline, FILE *flp, double tSecMin, double tSecMax);
static bool endsWith(const char *str, const char *postfix);

int main(int argc, char *argv[])
{
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <input.igc/.csv> <title> <output.flp>" << std::endl;
        exit(-1);
    }

    const char *inFile = &argv[1][0];

    bool isIgc = endsWith(inFile, ".igc") || endsWith(inFile, ".IGC");
    bool isCsv = endsWith(inFile, ".csv");

    if (!isIgc && !isCsv) {
        std::cerr << "Usage: " << argv[0] << " <input.igc/.csv> <title> <output.flp>" << std::endl;
        exit(-1);
    }

    // Create the output file.
    FILE *fOut = fopen(argv[3], "w");
    if (!fOut) {
        std::cerr << "Failed to create flight path file: " << argv[3] << std::endl;
        exit(-3);
    }

    // First set of header information.
    fprintf(fOut, "#Title     : %s\n", argv[2]);
    fprintf(fOut, "#Source    : %s\n", argv[3]);
    fprintf(fOut, "#Aircraft  : %s\n", "");

    if (isIgc) {
        std::cout << "Importing IGC file..." << std::endl;

        // Load the IGC file.
        IgcFile *fIn;
        IgcParser::parseIgcFile(inFile, &fIn);
        if (!fIn) {
            std::cerr << "Failed to parse IGC file: " << inFile << std::endl;
            exit(-2);
        }
        createFlightPathFileFromIgc(fIn, fOut);
    }

    if (isCsv) {
        std::cout << "Importing CSV file..." << std::endl;

        // Load the CSV file.
        std::ifstream csvFileStream(inFile);
        if (!csvFileStream.is_open()) {
            std::cerr << "Failed to parse CSV file: " << inFile << std::endl;
            exit(-2);
        }
        createFlightPathFileFromCsv(csvFileStream, fOut);
        csvFileStream.close();
    }

    std::cout << "Import complete." << std::endl;
    fclose(fOut);

    return 0;
}

static void createFlightPathFileFromIgc(IgcFile *igc, FILE *flp)
{
    double latMin = igc->latMin;
    double latMax = igc->latMax;
    double lonMin = igc->lonMin;
    double lonMax = igc->lonMax;

    printf("lat %f..%f, lon %f..%f\n", latMin, latMax, lonMin, lonMax);

    std::cout << "Reading raw data into spline..." << std::endl;
    Spline_CentripetalCatmullRom spline;

    double timestampSecMin = 0;
    double timestampSecMax = 0;

    int ix = 0;
    double oldTs = 0, olderTs = 0, oldestTs = 0;
    Vector3d *pOld = nullptr;
    int secCtr = 0;

    for (int iSec = igc->timestampMin; iSec < igc->timestampMax; iSec++) {

        if (++secCtr < SOURCE_INTERVAL_SEC) {
            continue;
        }

        if (igc->positions.count(iSec) <= 0) {
            continue;
        }
        // lon, lat in degrees, alt in meters
        IgcPosition *pos = igc->positions[iSec];

        Vector3d *p = new Vector3d(pos->lon, pos->lat, pos->altGps);
        if (pOld && (p->X() == pOld->X() && p->Y() == pOld->Y() && p->Z() == pOld->Z())) {
            continue;
        }
        pOld = p;

        secCtr = 0;

        spline.AppendPointAtTime(*p, iSec);
        // Remember the min and max timestamp.
        if (ix++ == 2) {
            timestampSecMin = iSec;
        }
        oldestTs = olderTs;
        olderTs = oldTs;
        oldTs = iSec;
    }
    timestampSecMax = oldestTs;

    printf("Imported IGC from time %lf to time %lf\n", timestampSecMin, timestampSecMax);

    saveFlightPath(spline, flp, timestampSecMin, timestampSecMax);
}

static void createFlightPathFileFromCsv(std::ifstream &csv, FILE *flp)
{
    std::string line;

    // Skip header line.
    getline(csv, line);

    // Parse data lines.
    std::cout << "Reading data into spline..." << std::endl;

    Spline_CentripetalCatmullRom spline;
    double timestampSecMin = 0;
    double timestampSecMax = 0;
    double oldTs = 0, olderTs = 0, oldestTs = 0;
    Vector3d *pOld = nullptr;
    int oldSec = 0;
    int ix = 0;

    while (getline(csv, line)) {
        if (line.size() == 0) {
            continue;
        }

        int hr = 0, min = 0, sec = 0;
        if (3 != sscanf(line.c_str(), "%d:%d:%d", &hr, &min, &sec)) {
            std::cerr << "Invalid time in line: " << line << std::endl;
            continue;
        }

        double lat = 0, lon = 0, alt = 0;
        if (3 != sscanf((const char*)&(line.c_str()[9]), "%lf,%lf,%lf", &lat, &lon, &alt)) {
            std::cerr << "Invalid position in line: " << line << std::endl;
            continue;
        }

        int timeSec = 3600 * hr + 60 * min + sec;

        // Skip some points.
        if (timeSec < (oldSec + 5)) {
            continue;
        }

        Vector3d *p = new Vector3d(lon, lat, alt);

        // Skip if unchanged.
        if (pOld && (p->X() == pOld->X() && p->Y() == pOld->Y() && p->Z() == pOld->Z())) {
            continue;
        }
        pOld = p;

        spline.AppendPointAtTime(*p, timeSec);
        // Remember the min and max timestamp.
        if (ix++ == 2) {
            timestampSecMin = timeSec;
        }
        oldestTs = olderTs;
        olderTs = oldTs;
        oldTs = timeSec;
    }
    timestampSecMax = oldestTs;

    saveFlightPath(spline, flp, timestampSecMin, timestampSecMax);
}

static void saveFlightPath(Spline_CentripetalCatmullRom &spline, FILE *flp, double tSecMin, double tSecMax)
{
    std::cout << "Writing interpolated data..." << std::endl;

    // Time resolution
    fprintf(flp, "#ResSec    : 1.0\n");

    double latDegStart = 0;
    double lonDegStart = 0;
    double altMtrStart = 0;
    bool hasStartValues = false;

    for (double iSec = tSecMin; iSec <= tSecMax; iSec += 1.0) {
        Vector3d *p = spline.InterpolatePointAtTime(iSec);

        double lonDeg = p->X();
        double latDeg = p->Y();
        double altMtr = p->Z();

        if (!hasStartValues) {
            lonDegStart = lonDeg;
            latDegStart = latDeg;
            altMtrStart = altMtr;
            hasStartValues = true;
            fprintf(flp, "#OriginSec : %f\n", tSecMin);
            fprintf(flp, "#OriginLoc : %lf;%lf;%lf\n", lonDeg, latDeg, altMtrStart);
        }

        // Lon (Deg), Lat (Deg), Alt (Mtr)
        fprintf(flp, "%.7lf;%.7lf;%.1lf\n", lonDeg, latDeg, altMtr);
    }
}

static bool endsWith(const char *str, const char *postfix)
{
    size_t sLen = strlen(str);
    size_t pfLen = strlen(postfix);
    for (int i = 0; i < pfLen; i++) {
        if (str[sLen - pfLen + i] != postfix[i]) {
            return false;
        }
    }
    return true;
}
