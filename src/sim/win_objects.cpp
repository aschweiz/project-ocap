//
// win_objects.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Object control window.
//
// 11.04.2024 ASR  First version
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

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <cmath>
#include <stdio.h>
#include <string>

#include "lon_lat_util.h"
#include "win_objects.h"
#include "workspace.h"

#include "Configuration.h"

static const int kCardHeight = 200;

using namespace gui;

static Win_Objects *sWinObjects;

Win_Objects *Win_Objects::Instance()
{
    if (!sWinObjects) {
        sWinObjects = new Win_Objects();
        sWinObjects->Create("Objects", 3005, 60, 820, 2120);
    }
    return sWinObjects;
}

Win_Objects::Win_Objects()
{
}

Win_Objects::~Win_Objects()
{
}

void Win_Objects::SetTestSet(TestSet *ts)
{
    testSet = ts;
    PostRedisplay();

    if (Workspace::Instance()->IsAutoRun()) {
        DeselectAll();
        FlightPathConfig *fpc = testSet->GetFlightPathAtIndex(0);
        if (fpc) {
            fpc->isSelected = true;
            Workspace::Instance()->PostRedisplayAll();
        }
    }
}

void Win_Objects::CreateSubclass()
{
}

void Win_Objects::SetTimeRange(TimeRange *tr)
{
    timeRange = tr;
    glutPostRedisplay();
}

void Win_Objects::Display()
{
    glClearColor(.9, .9, .9, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // --------------- 2D drawing ---------------

    SetProjection2D();

    if (testSet) {
        int yOff = 0;
        for (int i = 0, imax = testSet->GetCount(); i < imax; i++) {
            DisplayFlightPath(testSet->GetFlightPathAtIndex(i), yOff);
            yOff += kCardHeight;
        }
    }

    glutSwapBuffers();
}

void Win_Objects::Keyboard(unsigned char key, int x, int y)
{
    if (key == 0x1b) {
        // ESC: Deselect all flight paths in the test set.
        DeselectAll();
    }
}

void Win_Objects::Special(int key, int x, int y)
{
    std::string name;

    switch (key) {
    case GLUT_KEY_F1:
        name = "F1";
        break;
    case GLUT_KEY_LEFT:
        name = "Left";
        break;
    case 112:
        name = "Shift";
        break;
    default:
        name = "UNKONW";
        break;
    }
    printf("special: %d=%s %d,%d\n", key, name.c_str(), x, y);
}

void Win_Objects::SpecialUp(int key, int x, int y)
{
    std::string name;

    switch (key) {
    case GLUT_KEY_F1:
        name = "F1";
        break;
    case GLUT_KEY_LEFT:
        name = "Left";
        break;
    case 112:
        name = "Shift";
        break;
    default:
        name = "UNKONW";
        break;
    }
}

void Win_Objects::Mouse(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        DeselectAll();
        FlightPathConfig *fpc = GetFlightPathAtPosition(x, y);
        if (fpc) {
            fpc->isSelected = !fpc->isSelected;
            Workspace::Instance()->PostRedisplayAll();
        }
    }
}

FlightPathConfig *Win_Objects::GetFlightPathAtPosition(int x, int y)
{
    int ix = y / kCardHeight;
    if (!testSet || ix < 0 || ix >= testSet->GetCount()) {
        return nullptr;
    }
    return testSet->GetFlightPathAtIndex(ix);
}

void Win_Objects::DisplayFlightPath(FlightPathConfig *fpc, int yPos)
{
    int borderSize = 4;

    if (fpc->isSelected) {
        glColor3f(1.0, 1.0, 0.8);
    } else {
        glColor3f(1.0, 1.0, 1.0);
    }
    Rect r(borderSize, yPos + borderSize, width - borderSize, kCardHeight - borderSize);
    displayUtil->GlQuadRect(r, -0.001);

    glColor3f(0.0, 0.0, 0.0);

    double timestampSecMax = fpc->GetTimestampSecMax();
    double toSec = fpc->GetTimestampSecMin() + timeRange->GetTo();
    if (toSec > timestampSecMax) {
        toSec = timestampSecMax;
    }

    Vector3d vel;
    bool isVelValid = fpc->GetSpeedAtSec(toSec, &vel);

    // Identifier (Heli1, ...)
    DrawString2D(fpc->GetIdentifier().c_str(), 20, yPos + 50, 3.0);

    // Velocity
    char buf[256];
    if (isVelValid) {
        // 3-D velocity
        // double velLen = sqrt(vel.X() * vel.X() + vel.Y() * vel.Y() + vel.Z() * vel.Z());
        // sprintf(buf, "vel=%4.1fm/s (%4.1f; %4.1f; %4.1f)", velLen, vel.X(), vel.Y(), vel.Z());
        // 2-D velocity
        // Here we have lat=X / lon=Y!
        double vLatMs = vel.X();
        double vLonMs = vel.Y();
        double vAltMs = vel.Z();
        double vGroundMs = sqrt(vLatMs * vLatMs + vLonMs * vLonMs);
        double heading = -atan2(vLatMs, vLonMs) / M_PI * 180 + 90;
        if (heading > 180) {
            heading -= 360;
        }
        // Heading: 0=N -90=W +90=E 180=S
        sprintf(buf, "g=%.1f b=%.1f z=%.1f", vGroundMs, heading, vAltMs);
    } else {
        sprintf(buf, "vel=?");
    }
    DrawString2D(buf, 150, yPos + 50, 3.0);

    // Distance to selected objects.
    if (fpc->isSelected) {
        DrawString2D("(selected)", 150, yPos + 110, 3.0);
    } else {
        for (int i = 0, imax = testSet->GetCount(); i < imax; i++) {
            auto fpcOther = testSet->GetFlightPathAtIndex(i);
            if (fpcOther->isSelected) {
                Vector3d myPosDeg, otherPosDeg;
                if (fpc->GetPositionDegDegMtrAtMillisec(toSec * 1000, &myPosDeg)
                    && fpcOther->GetPositionDegDegMtrAtMillisec(toSec * 1000, &otherPosDeg)) {

                    Vector3d myPosMtr, otherPosMtr;
                    lonDegLatDegAltMtrToMtrMtrMtr(myPosDeg.GetVector(), myPosMtr.GetVector());
                    lonDegLatDegAltMtrToMtrMtrMtr(otherPosDeg.GetVector(), otherPosMtr.GetVector());

                    double distToOtherMtr = otherPosMtr.DistanceTo(myPosMtr);
                    sprintf(buf, "dist(%s)=%4.1fm", fpcOther->GetIdentifier().c_str(), distToOtherMtr);

                    double r = (distToOtherMtr <= 25) ? 1 : 0;
                    double g = 0;
                    if (distToOtherMtr <= 15) { // high
                        g = 0.0; // red
                    } else if (distToOtherMtr <= 20) { // med
                        g = 0.5; // orange
                    } else if (distToOtherMtr <= 25) { // low
                        g = 0.8; // yellow
                    }
                    DrawColorString2D(buf, 150, yPos + 110, 3.0, r, g, 0);
                }
            }
        }
    }

    // Alarm message
    double rgba[4];
    GetColorForAlarmLevel(fpc->alarmLevel, rgba);
    DrawColorString2D(fpc->alarmMessage.c_str(), 150, yPos + 170, 3.0, rgba[0], rgba[1], rgba[2]);
}

void Win_Objects::DeselectAll()
{
    if (testSet) {
        for (int i = 0, imax = testSet->GetCount(); i < imax; i++) {
            testSet->GetFlightPathAtIndex(i)->isSelected = false;
        }
        Workspace::Instance()->PostRedisplayAll();
    }
}
