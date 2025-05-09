//
// win_world.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Main window, displaying the simulated world.
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
#include "predicted_trace.h"
#include "win_world.h"
#include "workspace.h"

using namespace linalg;
using namespace gui;

static Win_World *sWinWorld;

Win_World *Win_World::Instance()
{
    if (!sWinWorld) {
        sWinWorld = new Win_World();
        sWinWorld->Create("Project OCAP - Simulation Viewer - Version 16.10.2024", 0, 60, 3000, 1550);
    }
    return sWinWorld;
}

Win_World::Win_World()
{
    displayUtil = new DisplayUtil();
}

Win_World::~Win_World()
{
    delete displayUtil;
}

void Win_World::CreateSubclass()
{
    // Subclass specific initialization.
    rotationDegXz = 60;
    rotationDegXy = -30;
    translationH = -0.2;
    translationV = -0.6;
    cameraRadius = 0.2;
    LookAt();
}

void Win_World::SetTimeRange(TimeRange *tr)
{
    timeRange = tr;
}

void Win_World::SetWorldCoords(WorldCoords *w)
{
    worldCoords = w;
    glutPostRedisplay();
}

void Win_World::SetTestSet(TestSet *ts)
{
    testSet = ts;

    // Apply the view configuration from the test set.
    rotationDegXz = ts->rotXZ;
    rotationDegXy = ts->rotXY;
    translationH = ts->trH;
    translationV = ts->trV;
    cameraRadius = ts->camR;
    LookAt();

    glutPostRedisplay();
}

void Win_World::Display()
{
    glClearColor(.9, .9, .9, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // --------------- Switch to 3D drawing ---------------

    SetProjection3D();

    glTranslated(translationH, 0, translationV);

    // Render flight paths and predicted traces.
    for (int i = 0, imax = testSet->GetCount(); i < imax; i++) {
        FlightPathConfig *fpc = testSet->GetFlightPathAtIndex(i);
        DrawFlightPath(fpc);

        // Render predicted traces for this aircraft.
        DisplayPredictedTraces(fpc);
    }

    // Now, so that alpha blending works.
    DrawBounds();

    // --------------- Switch to 2D drawing ---------------

    SetProjection2D();

    DrawString2D("test_set", 20, 50, 3.0);
    DrawString2D(testSet->GetName().c_str(), 250, 50, 3.0);

    char buf[256];
    sprintf(buf, "rotXZ=%d°, rotXY=%d°, trH=%.3lf, trV=%.3lf, camR=%.3lf",
        (int)rotationDegXz, (int)rotationDegXy, translationH, translationV, cameraRadius);
    DrawString2D(buf, 1000, 50, 3.0);

    DrawString2D("sim_time", 20, 110, 3.0);
    char simTimeStr[64];
    sprintf(simTimeStr, "%.1lf  %.3lf",
        timeRange->GetTo(),
        timeRange->GetTo() + testSet->GetStartTimeSec());
    DrawString2D(simTimeStr, 250, 110, 3.0);

    glutSwapBuffers();
}

void Win_World::Keyboard(unsigned char key, int x, int y)
{
    if (isprint(key)) {
        printf("key: `%c' %d,%d\n", key, x, y);
    } else {
        printf("key: 0x%x %d,%d\n", key, x, y);
    }

    if (key == '-') {
        cameraRadius = cameraRadius * 1.2;
        if (cameraRadius > 30.0) {
            cameraRadius = 30.0;
        }
        LookAt();
    } else if (key == '+') {
        cameraRadius = cameraRadius / 1.2;
        if (cameraRadius < 0.01) {
            cameraRadius = 0.01;
        }
        LookAt();
    }
}

void Win_World::Special(int key, int x, int y)
{
    std::string name;

    switch (key) {
    case GLUT_KEY_F1:
        name = "F1";
        break;
    case GLUT_KEY_LEFT:
        name = "Left";
        if (shiftClicked) {
            ChangeTranslation(-10.0, 0);
        } else {
            ChangeRotation(-10.0, 0);
        }
        break;
    case GLUT_KEY_RIGHT:
        name = "Right";
        if (shiftClicked) {
            ChangeTranslation(10.0, 0);
        } else {
            ChangeRotation(10.0, 0);
        }
        break;
    case GLUT_KEY_UP:
        name = "Up";
        if (shiftClicked) {
            ChangeTranslation(0, 10);
        } else {
            ChangeRotation(0, 10);
        }
        break;
    case GLUT_KEY_DOWN:
        name = "Down";
        if (shiftClicked) {
            ChangeTranslation(0, -10);
        } else {
            ChangeRotation(0, -10);
        }
        break;
    case GLUT_KEY_PAGE_UP:
        name = "Page up";
        break;
    case GLUT_KEY_PAGE_DOWN:
        name = "Page down";
        break;
    case GLUT_KEY_HOME:
        name = "Home";
        break;
    case GLUT_KEY_END:
        name = "End";
        break;
    case GLUT_KEY_INSERT:
        name = "Insert";
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

void Win_World::SpecialUp(int key, int x, int y)
{
    switch (key) {
    case 112:
    case 113:
        shiftClicked = false;
        break;
    default:
        break;
    }
}

void Win_World::Mouse(int button, int state, int x, int y)
{
}

void Win_World::ChangeTranslation(double deltaH, double deltaV)
{
    translationH += 0.01 * deltaH;
    translationV += 0.01 * deltaV;

    LookAt();
}

void Win_World::DrawBounds()
{
    GLUquadricObj *qobj = gluNewQuadric();
    gluQuadricDrawStyle(qobj, GLU_FILL);

    glColor4f(0.7, 0.7, 0.7, 0.2);

    for (int j = -20; j < 20; j++) {
        for (int i = -20; i < 20; i++) {
            if ((j % 2 == 0 && i % 2 != 0) || (j % 2 != 0 && i % 2 == 0)) {
                continue;
            }

            glBegin(GL_POLYGON);

            GLfloat x0 = 0.1 * i;
            GLfloat x1 = 0.1 * (i + 1);

            GLfloat z0 = 0.1 * j;
            GLfloat z1 = 0.1 * (j + 1);

            GLfloat y0 = 0; // alt = 0 mtr

            glVertex3f(x0, y0, z0);
            glVertex3f(x1, y0, z0);
            glVertex3f(x1, y0, z1);
            glVertex3f(x0, y0, z1);
            glEnd();
        }
    }

    glLineWidth(10.0);

    // x lon: red
    glColor4f(1.0, 0.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0, 0, 0); // lon  0
    glVertex3f(-1, 0, 0); // lon -1 (x direction)
    glEnd();

    // y lat: green
    glColor4f(0.0, 1.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0, 0, 0); // lat 0
    glVertex3f(0, 0, 1); // lat 1 (y direction)
    glEnd();
}

void Win_World::DrawFlightPath(FlightPathConfig *p)
{
    double lonWorld = 0, latWorld = 0, altWorld = 0; // -1..1
    double lonWorldOld = 0, latWorldOld = 0, altWorldOld = 0; // -1..1
    bool hasOld = false;

    Vector3d vecLonLatAltDeg(0, 0, 0);
    Vector3d vOld(0, 0, 0);
    bool hasPosition = false;

    if (!p) {
        return;
    }

    glPushMatrix();

    double timestampSecMin = p->GetTimestampSecMin();
    double timestampSecMax = p->GetTimestampSecMax();

    double fromSec = p->GetTimestampSecMin() + timeRange->GetFrom();
    double toSec = p->GetTimestampSecMin() + timeRange->GetTo();

    if (fromSec < timestampSecMin) {
        fromSec = timestampSecMin;
    }
    if (toSec > timestampSecMax) {
        toSec = timestampSecMax;
    }

    double colorFactor = 1.0 / (timestampSecMax - timestampSecMin);

    for (double iSec = (double)fromSec; iSec <= (double)toSec; iSec += 1.0) {
        long iMs = (long)iSec * 1000;
        hasPosition = p->GetPositionDegDegMtrAtMillisec(iMs, &vecLonLatAltDeg);
        if (!hasPosition) {
            continue;
        }

        double lineWidth = 6.0;
        if (p->HasPackets()) {
            int rssi = p->GetRssiAtSecond((int)iSec);
            if (rssi == -999) {
                lineWidth = 2.0;
            } else {
                lineWidth = 10.0;
            }
        }

        double rgba[4];
        EAlarmLevel alarmLevel = p->GetAlarmLevelAtSecond((int)iSec);
        GetColorForAlarmLevel(alarmLevel, &rgba[0]);

        if (alarmLevel == ALARM_LEVEL_NONE) {
            if (!p->isSelected) {
                rgba[0] = 0;
                rgba[1] = 0;
                rgba[2] = 0;
            } else {
                rgba[0] = 0;
                rgba[1] = 0.5;
                rgba[2] = 0;
            }
        }

        rgba[3] = 1.0 / (toSec - fromSec) * (iSec - fromSec);

        lonWorld = worldCoords->lonDegToWorld(vecLonLatAltDeg.X());
        latWorld = worldCoords->latDegToWorld(vecLonLatAltDeg.Y());
        altWorld = worldCoords->altMtrToWorld(vecLonLatAltDeg.Z());

        // y up, x inverted
        if (hasOld) {
            glLineWidth(lineWidth);
            glBegin(GL_LINES);

            glColor4f(rgba[0], rgba[1], rgba[2], rgba[3]);

            glVertex3f(lonWorldOld, altWorldOld, latWorldOld);
            glVertex3f(lonWorld, altWorld, latWorld);

            glEnd();
        }

        lonWorldOld = lonWorld;
        latWorldOld = latWorld;
        altWorldOld = altWorld;
        hasOld = true;
    }

    // Show the direction of the velocity vector and the arc center Z.
    if (p->hasZVec) { // && !p->isSelected) {
        TVector posVec;
        lonLatAltMtrToDegDegMtr(p->posVec.GetVector(), &posVec);
        float lonPos = worldCoords->lonDegToWorld(posVec.x);
        float latPos = worldCoords->latDegToWorld(posVec.y);
        float altPos = worldCoords->altMtrToWorld(posVec.z);

        // Display the Z vector.

        // In the linear model, we don't use Z.
        if (p->predictionModel != OCAP_PATH_MODEL_LINEAR) {
            TVector zVec;
            lonLatAltMtrToDegDegMtr(p->zVec.GetVector(), &zVec);
            float lonZ = worldCoords->lonDegToWorld(zVec.x);
            float latZ = worldCoords->latDegToWorld(zVec.y);
            float altZ = worldCoords->altMtrToWorld(zVec.z);

            glLineWidth(4);
            glBegin(GL_LINES);
            if (p->predictionModel == OCAP_PATH_MODEL_ARC) {
                glColor4f(0, 1, 1, 1);
            } else if (p->predictionModel == OCAP_PATH_MODEL_SPHERIC) {
                glColor4f(1, 1, 0, 1);
            }
            glVertex3f(lonPos, altPos, latPos);
            glVertex3f(lonZ, altZ, latZ);
            glEnd();
        }

        // Display the velocity vector.

        Vector3d velEndVecMtr = p->posVec;
        Vector3d velVecMtr = p->velVec;
        velVecMtr = velVecMtr * 50;
        velEndVecMtr = velEndVecMtr + velVecMtr;
        TVector velEndVec;
        lonLatAltMtrToDegDegMtr(velEndVecMtr.GetVector(), &velEndVec);
        float lonVel = worldCoords->lonDegToWorld(velEndVec.x);
        float latVel = worldCoords->latDegToWorld(velEndVec.y);
        float altVel = worldCoords->altMtrToWorld(velEndVec.z);

        glLineWidth(2);
        glBegin(GL_LINES);
        if (p->predictionModel == OCAP_PATH_MODEL_LINEAR) {
            glColor4f(1, 0, 1, 1);
        } else {
            glColor4f(.5, .5, 0, 1);
        }
        glVertex3f(lonPos, altPos, latPos);
        glVertex3f(lonVel, altVel, latVel);
        glEnd();
    }

    glTranslated(lonWorld, altWorld + 0.02, latWorld);
    glScalef(0.0002, 0.0002, 0.0002);
    // Rotate characters back into screen plane.
    glRotatef(rotationDegXz, 0, 1.0, 0);
    DrawString(p->GetIdentifier().c_str(), 2);

    glPopMatrix();
}

void Win_World::DisplayPredictedTraces(FlightPathConfig *p)
{
    glPushMatrix();
    glLineWidth(1.5);

    int imax = p->GetNofPredictedTraces();
    for (int i = 1; i < imax; i++) {
        PredictedTrace *pt = p->GetPredictedTraceAtIndex(i);

        glBegin(GL_LINE_STRIP);
        for (int j = 0; j < T_MAX_SEC; j++) {

            double colorRed = 0.0;
            double colorGreen = 0.0;
            double colorBlue = 0.0;

            // Dim "old" traces.
            double colorAlpha = 1.0 - (imax - i) * 0.1;
            if (colorAlpha < 0.1) {
                colorAlpha = 0.1;
            }

            // Show seconds raster.
            if (j % 4 > 1) {
                colorGreen = 1.0;
            } else {
                colorBlue = 1.0;
            }

            glColor4f(colorRed, colorGreen, colorBlue, colorAlpha);

            Vector3d *vMtr = &pt->positionsMtr[j];
            Vector3d vDeg;
            lonLatAltMtrToDegDegMtr(vMtr->GetVector(), vDeg.GetVector());

            // Convert from Deg/Deg/Mtr to world coordinates (-1..1).
            double lonWorld = worldCoords->lonDegToWorld(vDeg.X());
            double latWorld = worldCoords->latDegToWorld(vDeg.Y());
            double altWorld = worldCoords->altMtrToWorld(vDeg.Z());
            glVertex3f(lonWorld, altWorld, latWorld);
        }
        glEnd();
    }
    glPopMatrix();
}
