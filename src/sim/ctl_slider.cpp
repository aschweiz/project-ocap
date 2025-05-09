//
// ctl_slider.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Simple OpenGL slider control for the OCAP simulation environment.
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

#include <stdio.h>

#include "ctl_slider.h"
#include "display_util.h"

#include "workspace.h"

using namespace gui;

CtlSlider::CtlSlider()
{
    displayUtil = new DisplayUtil();
}

CtlSlider::~CtlSlider()
{
    delete displayUtil;
}

void CtlSlider::SetTimeRange(TimeRange *tr)
{
    timeRange = tr;
    glutPostRedisplay();
}

TimeRange *CtlSlider::GetTimeRange()
{
    return timeRange;
}

void CtlSlider::Display(int width, int height)
{
    displayUtil->SetWidthHeight(width, height);

    int sliderY = 150;
    int sliderH = 10;

    int sliderBtnW = 25;
    int sliderBtnH = 50;

    glColor4f(0.7, 0.7, 0.7, 1.0);
    int bl = 20, br = 20;
    int w0 = width - bl - br;

    sliderStartX = bl;
    sliderEndX = sliderStartX + w0;
    displayUtil->GlQuadXywh(sliderStartX, sliderY - sliderH / 2, w0, sliderH, -0.002);

    int dxFrom = 0;
    int dxTo = w0;

    if (timeRange) {
        double ppw = w0 / timeRange->GetFullRange();
        dxFrom = (int)(ppw * timeRange->GetFrom());
        dxTo = (int)(ppw * timeRange->GetTo());
    }

    fromRect.x = sliderStartX + dxFrom - (sliderBtnW / 2);
    fromRect.y = sliderY - (sliderBtnH / 2);
    fromRect.w = sliderBtnW;
    fromRect.h = sliderBtnH;

    toRect.x = sliderStartX + dxTo - (sliderBtnW / 2);
    toRect.y = sliderY - (sliderBtnH / 2);
    toRect.w = sliderBtnW;
    toRect.h = sliderBtnH;

    // Selected part of the slider, between knobs.
    if (draggingFromRect || draggingToRect) {
        glColor4f(0.7, 1.0, 0.5, 1.0);
    } else {
        glColor4f(0.4, 0.6, 0.9, 1.0);
    }
    displayUtil->GlQuadXywh(
        fromRect.x,
        sliderY - sliderH / 2,
        (toRect.x - fromRect.x),
        sliderH,
        -0.001);

    // Dashes
    glColor4f(0.7, 0.7, 0.7, 1.0);
    glLineWidth(4.0);
    int nofDashes = 20;
    for (int i = 1; i < nofDashes; i++) {
        int ix = sliderStartX + (sliderEndX - sliderStartX) * i / nofDashes;
        glBegin(GL_LINES);
        glVertex2d(ix, sliderY + 30);
        int y2 = (i % 2) == 0 ? 65 : 45;
        glVertex2d(ix, sliderY + y2);
        glEnd();
    }

    // From rect.
    if (draggingFromRect) {
        glColor4f(0.7, 1.0, 0.5, 1.0);
    } else {
        glColor4f(0.4, 0.6, 0.9, 1.0);
    }
    displayUtil->GlQuadRect(fromRect, 0);

    // To rect.
    if (draggingToRect) {
        glColor4f(0.7, 1.0, 0.5, 1.0);
    } else {
        glColor4f(0.4, 0.6, 0.9, 1.0);
    }
    displayUtil->GlQuadRect(toRect, 0);
}

void CtlSlider::Mouse(int button, int state, int x, int y)
{
    if (button == 0) {
        if (state == GLUT_UP) {
            // Released left button.
            draggingFromRect = false;
            draggingToRect = false;
            Workspace::Instance()->PostRedisplayAll();
        } else {
            // Pressed left button.
            if (toRect.ContainsPoint(x, y)) {
                if (!draggingToRect && state == GLUT_DOWN) {
                    draggingToRect = true;
                    Workspace::Instance()->PostRedisplayAll();
                }
            } else if (fromRect.ContainsPoint(x, y)) {
                if (!draggingFromRect && state == GLUT_DOWN) {
                    draggingFromRect = true;
                    Workspace::Instance()->PostRedisplayAll();
                }
            }
        }
    }
}

void CtlSlider::Motion(int x, int y)
{
    // Only interesting if we're dragging a slider.
    if (!draggingFromRect && !draggingToRect) {
        return;
    }

    // Our origin is bottom-left.
    y = displayUtil->GetHeight() - y;

    // Make sure we're in the specified range.
    if (x < sliderStartX) {
        x = sliderStartX;
    } else if (x > sliderEndX) {
        x = sliderEndX;
    }

    int maxDeltaX = sliderEndX - sliderStartX;
    int curDeltaX = x - sliderStartX;

    double pos = timeRange->GetFullRange() * curDeltaX / maxDeltaX;

    if (draggingFromRect) {
        timeRange->SetFrom(pos, false);
    } else if (draggingToRect) {
        timeRange->SetTo(pos, false);
    }

    Workspace::Instance()->PostRedisplayAll();
}
