//
// parse_pflag.c
//
// OCAP - Open Collision Avoidance Protocol
//
// Parser for recorded PFLAG messages.
//
// 05.11.2024 ASR  First version
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

#include "parse_pflag.h"
#include <math.h>

int strParseDecimal(const char **inStr, int *outValue, int nexp);
int strParseToken(const char **inStr, char *outToken, int bufSize);

int pflaDecodeG_A(const char *pflag, TVector *pos, TVector *vel)
{
    pflag = &pflag[7]; // A,2022

    // Example:
    // $PFLAG,A,2022,06,17,110716,4656.83361,N,00727.18699,E,585.7,429.5,0.0,358.0,-0.0,1.6,1.1,0*34
    int yr = 0, mt = 0, dy = 0, hourMinSec = 0;
    int latE5, lonE5, alt1E1, alt2E1, velE2, courseE1, vvelE2, haccME2, vaccME2, gspeedAccMsE2;
    char latNS, lonEW;

    const char *str = &pflag[2];
    int n = strParseDecimal(&str, &yr, 0);
    n += strParseDecimal(&str, &mt, 0);
    n += strParseDecimal(&str, &dy, 0);
    n += strParseDecimal(&str, &hourMinSec, 0);

    // 06.04.2022  Ignore the 0.0.2000 packets, otherwise time would be stuck...
    if (yr == 2000) {
        return 1;
    }

    // Next fields:
    // 4624.99377,N,00806.33401,E,
    // GPS latitude and longitude, 4624.9 is 46 degrees, 24.9 arc minutes

    // 46
    int degrees;
    char degreeBuf[5];
    degreeBuf[0] = *str++;
    degreeBuf[1] = *str++;
    degreeBuf[2] = 0;
    const char *p = &degreeBuf[0];
    n += strParseDecimal(&p, &degrees, 0);

    // 24.99377 -> 2499377
    n += strParseDecimal(&str, &latE5, 5);
    latE5 = (latE5 * 100) / 6000 + degrees * 100000;
    latNS = *str++; // N/S
    str++; // ,

    degreeBuf[0] = *str++;
    degreeBuf[1] = *str++;
    degreeBuf[2] = *str++;
    degreeBuf[3] = 0;
    p = &degreeBuf[0];
    n += strParseDecimal(&p, &degrees, 0);

    n += strParseDecimal(&str, &lonE5, 5);
    lonE5 = (lonE5 * 100) / 6000 + degrees * 100000;
    lonEW = *str++; // E/W
    str++; // ,

    // Next fields:
    // Height (m) from GPS and barometric
    // 2294.3,2161.7,
    n += strParseDecimal(&str, &alt1E1, 1);
    n += strParseDecimal(&str, &alt2E1, 1);

    // Next fields:
    // velocity (m/s), course (degrees, 0=N?, 90=E?), vertical velocity (m/s? pos=up?)
    // 5.5,163.1,-1.6,
    n += strParseDecimal(&str, &velE2, 2); // cm/s
    n += strParseDecimal(&str, &courseE1, 1); // dezideg
    n += strParseDecimal(&str, &vvelE2, 2); // cm/s

    // Next fields:
    // haccM, vaccM, gspeedAccMs
    n += strParseDecimal(&str, &haccME2, 2);
    n += strParseDecimal(&str, &vaccME2, 2);
    n += strParseDecimal(&str, &gspeedAccMsE2, 2);

    if (n != 16) {
        return 0;
    }

    uint32_t ownDateYrMtDy = 10000 * yr + 100 * mt + dy;
    if (ownDateYrMtDy < 20220101 || ownDateYrMtDy > 20991231
        || hourMinSec > 235959) {
        return 0;
    }

    latE5 = latE5 * (latNS != 'N' ? -1 : +1);
    lonE5 = lonE5 * (lonEW != 'E' ? -1 : +1);

    pos->x = 0.00001f * lonE5;
    pos->y = 0.00001f * latE5;
    pos->z = 0.1f * alt1E1;

    float velXyMs = .01f * velE2;
    float headingRad = (0.1f * courseE1) * 3.14159265f / 180.0f; // N=0, E=90, ...
    vel->x = velXyMs * sinf(headingRad);
    vel->y = velXyMs * cosf(headingRad);
    vel->z = .01f * vvelE2;

    return 1;
}

int strParseDecimal(const char **inStr, int *outValue, int nexp)
{
    if (!inStr || !outValue) {
        return 0;
    }

    char buf[12];
    int result = 0;
    char *bufStr = &buf[0];
    char c;

    int success = strParseToken(inStr, &buf[0], 12);
    if (!success || !buf[0]) {
        return 0;
    }
    // Ensure the input string is properly terminated.
    buf[11] = 0x00;
    // Skip + and - sign at the beginning.
    c = buf[0];
    if (c == '+' || c == '-') {
        bufStr++;
    }
    // Construct the number from the individual digits.
    int inExp = 0;
    while ((c = *bufStr)) {
        // Start of fractional part?
        if (c == '.') {
            if (inExp > 0) {
                // Was already in fractional part...
                return 0;
            }
            inExp = 1;
            bufStr++;
            continue;
        }
        // Only digits are valid.
        if (c < '0' || c > '9') {
            return 0;
        }
        if (inExp <= nexp) {
            result = 10 * result + (c - '0');
        }
        if (inExp > 0) {
            inExp++;
        }
        bufStr++;
    }
    // If there was no decimal point, we simulate one so that the fractional part is filled.
    if (inExp == 0 && nexp > 0) {
        inExp = 1;
    }
    // Complete the scaling, e.g. 2.75 -> 2750 for nexp=3.
    while (inExp > 0 && inExp <= nexp) {
        result = 10 * result;
        inExp++;
    }
    *outValue = (buf[0] == '-') ? -result : result;
    return 1;
}

int strParseToken(const char **inStr, char *outToken, int bufSize)
{
    if (!inStr || !outToken) {
        return 0;
    }

    const char *str = *inStr;
    while (1) {
        char c = *str;
        // Character consumed, advance to next.
        if (c) {
            str++;
        }
        // End of token or string (implies end of token)?
        if (!c || c == ',' || c == '*' || c == '\r' || c == '\n') {
            *inStr = str;
            // Enough space to put the string terminator?
            if (bufSize > 0) {
                *outToken = 0;
                return 1;
            }
            return 0;
        }
        // Space to copy the current character?
        if (bufSize > 0) {
            *outToken++ = c;
            bufSize--;
        }
    }
}
