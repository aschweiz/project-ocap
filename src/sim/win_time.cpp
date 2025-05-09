//
// win_time.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Time control window.
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

#include "win_time.h"
#include "workspace.h"

using namespace gui;

static Win_Time *sWinTime;
static bool sTimerFuncActive;

static void WinTimeGlutTimerFunc(int value)
{
    if (!sTimerFuncActive) {
        return;
    }

    sWinTime->TimerFunc(value);
}

Win_Time *Win_Time::Instance()
{
    if (!sWinTime) {
        sWinTime = new Win_Time();
        sWinTime->Create("Time Control", 0, 130 + 1550, 3000, 500);
    }
    return sWinTime;
}

Win_Time::Win_Time()
{
    timeSlider = new CtlSlider();

    if (Workspace::Instance()->IsAutoRun()) {
        sTimerFuncActive = true;
        glutTimerFunc(3000, WinTimeGlutTimerFunc, 0);
    }
}

Win_Time::~Win_Time()
{
    sTimerFuncActive = false;
    delete timeSlider;
}

void Win_Time::CreateSubclass()
{
    // Subclass specific initialization.
    PostRedisplay();
}

void Win_Time::SetTimeRange(TimeRange *tr)
{
    timeSlider->SetTimeRange(tr);
    glutPostRedisplay();
}

void Win_Time::SetTestSet(TestSet *ts)
{
    testSet = ts;
}

void Win_Time::Display()
{
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // --------------- 2D drawing ---------------

    SetProjection2D();

    glColor4f(0.0, 0.0, 0.0, 1.0);
    DrawString2D("Set simulation time for analysis", 20, 50, 3.0);

    timeSlider->Display(width, height);

    // (p) Replay recorded packets

    DrawString2D("(p)", 20, 300, 3.0);
    char buf[256];
    sprintf(buf, "Replay recorded packets (currently %s)",
        Workspace::Instance()->GetReplayPackets() ? "ON" : "OFF");
    DrawString2D(buf, 120, 300, 3.0);

    // (e) Use iConspicuity extensions

    DrawString2D("(e)", 20, 350, 3.0);
    sprintf(buf, "Use iConspicuity extensions (currently %s)",
        Workspace::Instance()->GetUseIConspicuity2() ? "ON" : "OFF");
    DrawString2D(buf, 120, 350, 3.0);

    glutSwapBuffers();
}

void Win_Time::Keyboard(unsigned char key, int x, int y)
{
    if (isprint(key)) {
        printf("key: `%c' %d,%d\n", key, x, y);
    } else {
        printf("key: 0x%x %d,%d\n", key, x, y);
    }

    // Run the collision prediction if the user clicks space.
    if (key == ' ') {
        if (!Workspace::Instance()->IsCollisionWarningRunning()) {
            Workspace::Instance()->StartCollisionWarning();
        } else {
            Workspace::Instance()->StopCollisionWarning();
        }
    }

    // Toggling packet replay (for flight paths where they are available).
    if (key == 'p') {
        Workspace::Instance()->SetReplayPackets(
            !Workspace::Instance()->GetReplayPackets());
    }

    // Toggling between iConspicuity and iConspicuity2 formats.
    if (key == 'e') {
        Workspace::Instance()->SetUseIConspicuity2(
            !Workspace::Instance()->GetUseIConspicuity2());
    }
}

void Win_Time::Special(int key, int x, int y)
{
    std::string name;
    TimeRange *tr = timeSlider->GetTimeRange();
    double toSec = tr->GetTo();

    switch (key) {
    case GLUT_KEY_F1:
        name = "F1";
        break;
    case GLUT_KEY_LEFT:
        name = "Left";
        if (shiftClicked) {
            tr->SetTo(toSec - 1, false);
        } else {
            tr->SetTo(toSec - .1, false);
        }
        Workspace::Instance()->PostRedisplayAll();
        break;
    case GLUT_KEY_RIGHT:
        name = "Right";
        if (shiftClicked) {
            tr->SetTo(toSec + 1, false);
        } else {
            tr->SetTo(toSec + .1, false);
        }
        Workspace::Instance()->PostRedisplayAll();
        break;
    case 112:
    case 113:
        name = "Shift";
        shiftClicked = true;
        break;
    default:
        name = "UNKONW";
        break;
    }
}

void Win_Time::SpecialUp(int key, int x, int y)
{
    std::string name;

    switch (key) {
    case GLUT_KEY_F1:
        name = "F1";
        break;
    case GLUT_KEY_LEFT:
        name = "Left";
        break;
    case GLUT_KEY_RIGHT:
        name = "Right";
        break;
    case 112:
    case 113:
        shiftClicked = false;
        name = "Shift";
        break;
    default:
        name = "UNKONW";
        break;
    }
}

void Win_Time::Mouse(int button, int state, int x, int y)
{
    // y is from bottom!
    timeSlider->Mouse(button, state, x, y);
}

void Win_Time::Motion(int x, int y)
{
    timeSlider->Motion(x, y);
}

void Win_Time::EnterLeave(int state)
{
}

void Win_Time::TimerFunc(int value)
{
    TimeRange *tr = timeSlider->GetTimeRange();
    double toSec = tr->GetTo();

    if (timerCtr == 0) {
        tr->SetFrom(testSet->GetStartTimeAutoRunSec(), 0);
        tr->SetTo(testSet->GetStartTimeAutoRunSec() + 1, 0);
        Workspace::Instance()->StartCollisionWarning();
        Workspace::Instance()->PostRedisplayAll();

    } else if (toSec < testSet->GetEndTimeAutoRunSec()) {
        tr->SetTo(toSec + 1, false);
        Workspace::Instance()->PostRedisplayAll();

    } else {
        Workspace::Instance()->StopCollisionWarning();
        exit(0);
    }

    timerCtr++;

    unsigned int timeMillisec = 10;
    if (Workspace::Instance()->IsRealTime()) {
        timeMillisec = 1000;
    }
    glutTimerFunc(timeMillisec, WinTimeGlutTimerFunc, 0);
}
