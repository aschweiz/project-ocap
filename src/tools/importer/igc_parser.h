//
// igc_parser.h
//
// OCAP - Open Collision Avoidance Protocol
//
// Simple parser for IGC files.
//
// 07.11.2023 ASR  First version
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

#include <string>
#include <map>

#ifndef __IGC_PARSER_H__
#define __IGC_PARSER_H__ 1

class IgcPosition {
public:
    int timestamp;   // 080058
    double lat;      // 47.1234
    double lon;      //  8.1234
    double altGps;   // 561.0
    double altBaro;  // 512.0
};

class IgcFile {
public:
    int timestampMin;
    int timestampMax;
    double latMin;
    double latMax;
    double lonMin;
    double lonMax;
    std::map<int, IgcPosition*> positions;
};

class IgcSet {
public:
    double getLatMin();
    double getLatMax();
    double getLonMin();
    double getLonMax();

public:
    std::map<std::string, IgcFile*> files;
};

class IgcParser {
public:
    static int parseIgcFile(std::string path, IgcFile **file);
};

#endif // __IGC_PARSER_H__
