//
// win_base.h
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

#ifndef __WIN_BASE_H__
#define __WIN_BASE_H__ 1

#include "AlarmState.h"
#include "display_util.h"
#include "linalg.h"

namespace gui {

class Win_Base {
protected:
    Win_Base();

public:
    virtual ~Win_Base();

    void Create(const char *title, int posX, int posY, int w, int h);

public:
    virtual void CreateSubclass();

    virtual void Reshape(int w, int h);
    virtual void Display() = 0;
    virtual void Keyboard(unsigned char key, int x, int y);
    virtual void Special(int key, int x, int y);
    virtual void SpecialUp(int key, int x, int y);
    virtual void Mouse(int button, int state, int x, int y);
    virtual void Motion(int x, int y);
    virtual void EnterLeave(int state);
    virtual void Idle();

    virtual void PostRedisplay();
    virtual void SetProjection2D();
    virtual void SetProjection3D();
    virtual void LookAt();
    virtual void ChangeRotation(double deltaXz, double deltaXy);

    virtual void DrawColorString2D(const char *str, int x, int y, double lineWidth, double r, double g, double b);
    virtual void DrawString2D(const char *str, int x, int y, double lineWidth);
    virtual void DrawColorString(const char *str, double lineWidth, double r, double g, double b);
    virtual void DrawString(const char *str, double lineWidth);

    virtual void GetColorForAlarmLevel(EAlarmLevel l, double *rgba);

protected:
    int width = 100;
    int height = 100;
    DisplayUtil *displayUtil = nullptr;

    double cameraRadius = 5.0;
    double rotationDegXz = 0;
    double rotationDegXy = 0;

private:
    int window = -1;

    linalg::Vector3d eyePositionVector;
};

}

#endif // __WIN_BASE_H__
