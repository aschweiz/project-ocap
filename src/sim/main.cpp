//
// main.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// OCAP simulation and testing environment.
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

#include <string.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "workspace.h"

static void printUsageAndExit(const char *appName);

int main(int argc, char *argv[])
{
    if (argc < 2) {
        printUsageAndExit(argv[0]);
    }

    int fileNameIx = 1;

    glutInit(&argc, argv);

    // Create the workspace.
    int isAutoRun = !strcmp(argv[1], "--autorun");
    int isAutoRunRealTime = !strcmp(argv[1], "--autorun-realtime");
    std::string outputFileName = "prediction_result.txt";
    if (isAutoRun || isAutoRunRealTime) {
        if (argc < 3) {
            printUsageAndExit(argv[0]);
        }
        fileNameIx++;
        if (argc >= 4) {
          outputFileName = argv[3];
        }
    }

    Workspace::Instance()->Initialize(
        argv[fileNameIx],
        isAutoRun || isAutoRunRealTime,
        isAutoRunRealTime,
        outputFileName);

    glutMainLoop();
    return 0;
}

void printUsageAndExit(const char *appName)
{
    fprintf(stderr, "usage: %s [--autorun] <.tst> [<.txt>]\n", appName);
    fprintf(stderr, "Runs the OCAP simulation environment.\n");
    fprintf(stderr, "Use autorun to execute a test case and exit afterwards.\n");
    fprintf(stderr, "Optionally specify a target file name for the result file.\n");
    exit(-1);
}
