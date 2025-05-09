//
// display_util.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// OpenGL display utilities for the OCAP simulation environment.
//
// 16.04.2024 ASR  First version
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

#include "display_util.h"
#include <stdio.h>

#define STROKE_BUF_SIZE 256

using namespace gui;

DisplayUtil::DisplayUtil()
{
}

DisplayUtil::~DisplayUtil()
{
}

void DisplayUtil::SetWidthHeight(int w, int h)
{
    width = w;
    height = h;
}

void DisplayUtil::GlQuadRect(Rect &r, double z)
{
    GlQuadXywh(r.x, r.y, r.w, r.h, z);
}

void DisplayUtil::GlQuadXywh(double x, double y, double w, double h, double z)
{
    glBegin(GL_QUADS);
    glVertex3f(x, y, z);
    glVertex3f(x + w, y, z);
    glVertex3f(x + w, y + h, z);
    glVertex3f(x, y + h, z);
    glEnd();
}

int DisplayUtil::GetHeight()
{
    return height;
}

int DisplayUtil::GetWidth()
{
    return width;
}

void DisplayUtil::StrokeOutput(double x, double y, const char *format, ...)
{
    va_list args;
    char buffer[STROKE_BUF_SIZE], *p;

    va_start(args, format);
    vsnprintf(buffer, STROKE_BUF_SIZE, format, args);
    va_end(args);

    glPushMatrix();
    glTranslatef(x, y, 0);
    glScalef(0.0003, 0.0012, 1);
    for (p = buffer; *p; p++) {
        glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
    }
    glPopMatrix();
}
