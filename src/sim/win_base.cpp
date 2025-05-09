//
// win_base.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Base class for OpenGL windows.
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

#include "win_base.h"
#include <cmath>
#include <unordered_map>

using namespace gui;

static std::unordered_map<int, Win_Base*> sAllWindows;

static void DisplayWinBase(void)
{
    Win_Base *win = sAllWindows[glutGetWindow()];
    win->Display();
}

static void ReshapeWinBase(int w, int h)
{
    Win_Base *win = sAllWindows[glutGetWindow()];
    win->Reshape(w, h);
}

static void KeyboardWinBase(unsigned char key, int x, int y)
{
    Win_Base *win = sAllWindows[glutGetWindow()];
    win->Keyboard(key, x, y);
}

static void SpecialWinBase(int key, int x, int y)
{
    Win_Base *win = sAllWindows[glutGetWindow()];
    win->Special(key, x, y);
}

static void SpecialUpWinBase(int key, int x, int y)
{
    Win_Base *win = sAllWindows[glutGetWindow()];
    win->SpecialUp(key, x, y);
}

static void MouseWinBase(int button, int state, int x, int y)
{
    Win_Base *win = sAllWindows[glutGetWindow()];
    win->Mouse(button, state, x, y);
}

static void MotionWinBase(int x, int y)
{
    Win_Base *win = sAllWindows[glutGetWindow()];
    win->Motion(x, y);
}

static void EnterLeaveWinBase(int state)
{
    Win_Base *win = sAllWindows[glutGetWindow()];
    win->EnterLeave(state);
}

static void IdleWinBase(void)
{
    Win_Base *win = sAllWindows[glutGetWindow()];
    win->Idle();
}

Win_Base::Win_Base()
{
    displayUtil = new DisplayUtil();
}

Win_Base::~Win_Base()
{
    delete displayUtil;
}

void Win_Base::Create(const char *title, int posX, int posY, int w, int h)
{
    // Define the display mode for windows that will be created afterwards.
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

    width = w;
    height = h;
    glutInitWindowSize(width, height);
    glutInitWindowPosition(posX, posY);

    window = glutCreateWindow(title);
    sAllWindows[window] = this;

    // Callback for re-drawing the content.
    glutDisplayFunc(DisplayWinBase);

    // Callback if the window size changes.
    glutReshapeFunc(ReshapeWinBase);

    // Keyboard input.
    glutKeyboardFunc(KeyboardWinBase);

    // Secial key handling.
    glutSpecialFunc(SpecialWinBase);
    glutSpecialUpFunc(SpecialUpWinBase);

    // Mouse input.
    glutMouseFunc(MouseWinBase);

    // For mouse movements in the window while a button is pressed.
    glutMotionFunc(MotionWinBase);

    glutEntryFunc(EnterLeaveWinBase);

    //    glutVisibilityFunc(visible);

    CreateSubclass();

    // Use alpha channel.
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    Reshape(width, height);
    LookAt();
}

void Win_Base::CreateSubclass()
{
    // Overridden by subclass.
}

void Win_Base::Reshape(int w, int h)
{
    width = w;
    height = h;
    glViewport(0, 0, w, h);
}

void Win_Base::SetProjection2D()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0f, width, height, 0.0f, 0.0f, 1.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void Win_Base::SetProjection3D()
{
    // Perspective
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, // FoV degrees
        (GLfloat)width / (GLfloat)height, // aspect ratio
        .1, // Z near
        1000.0 // Z far
    );

    // Look at
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -1.0);
    gluLookAt(
        eyePositionVector.X(),
        eyePositionVector.Y(),
        eyePositionVector.Z(), // eye is at (...)
        0.0,
        0.0,
        0.0, // center is at (0,0,0)
        0.0,
        1.0,
        0.0 // up is in positive Y direction
    );
}

void Win_Base::LookAt()
{
    GLfloat r = cameraRadius;

    // Height is constant during rotation.
    GLfloat y = r * sin(-rotationDegXy / 180.0 * M_PI);
    GLfloat f = cos(rotationDegXy / 180.0 * M_PI);

    GLfloat x = f * r * sin(rotationDegXz / 180.0 * M_PI);
    GLfloat z = f * r * cos(rotationDegXz / 180.0 * M_PI);

    eyePositionVector.Set(x, y, z);

    glutPostRedisplay();
}

void Win_Base::ChangeRotation(double deltaXz, double deltaXy)
{
    rotationDegXz += deltaXz;
    if (rotationDegXz < 0) {
        rotationDegXz += 360.0;
    } else if (rotationDegXz >= 360.0) {
        rotationDegXz -= 360.0;
    }

    rotationDegXy += deltaXy;
    if (rotationDegXy >= 90.0) {
        rotationDegXy = 90.0;
    } else if (rotationDegXy < -90.0) {
        rotationDegXy = -90.0;
    }

    LookAt();
}

void Win_Base::Keyboard(unsigned char key, int x, int y)
{
}

void Win_Base::Special(int key, int x, int y)
{
}

void Win_Base::SpecialUp(int key, int x, int y)
{
}

void Win_Base::Mouse(int button, int state, int x, int y)
{
}

void Win_Base::Motion(int x, int y)
{
}

void Win_Base::EnterLeave(int state)
{
}

void Win_Base::Idle()
{
}

void Win_Base::PostRedisplay()
{
    glutPostWindowRedisplay(window);
}

void Win_Base::DrawString2D(const char *str, int x, int y, double lineWidth)
{
    DrawColorString2D(str, x, y, lineWidth, 0, 0, 0);
}

void Win_Base::DrawColorString2D(const char *str, int x, int y, double lineWidth, double r, double g, double b)
{
    glPushMatrix();

    glTranslated(x, y, 0);
    glScalef(0.3, 0.3, 0);
    glRotatef(180, 1.0, 0.0, 0);

    DrawColorString(str, 3, r, g, b);

    glPopMatrix();
}

void Win_Base::DrawString(const char *str, double lineWidth)
{
    DrawColorString(str, lineWidth, 0, 0, 0);
}

void Win_Base::DrawColorString(const char *str, double lineWidth, double r, double g, double b)
{
    glColor4f(r, g, b, 1.0);
    glLineWidth(lineWidth);
    const char *p = nullptr;
    for (p = str; *p; p++) {
        glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
    }
}

void Win_Base::GetColorForAlarmLevel(EAlarmLevel l, double *rgba)
{
    double r = (l >= ALARM_LEVEL_1) ? 1 : 0;
    double g = 0;
    if (l == ALARM_LEVEL_3) { // high
        g = 0.0; // red
    } else if (l == ALARM_LEVEL_2) { // med
        g = 0.5; // orange
    } else if (l == ALARM_LEVEL_1) { // low
        g = 0.8; // yellow
    }

    rgba[0] = r;
    rgba[1] = g;
    rgba[2] = 0;
    rgba[3] = 1;
}
