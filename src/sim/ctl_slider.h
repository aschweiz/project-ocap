//
// ctl_slider.h
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

#ifndef __CTL_SLIDER_H__
#define __CTL_SLIDER_H__ 1

#include "display_util.h"
#include "rect.h"
#include "time_range.h"

namespace gui {

class CtlSlider {
public:
    CtlSlider();
    virtual ~CtlSlider();

    void SetTimeRange(TimeRange *tr);
    TimeRange *GetTimeRange();

    void Display(int width, int height);
    void Mouse(int button, int state, int x, int y);
    void Motion(int x, int y);

private:
    DisplayUtil *displayUtil = nullptr;

    Rect fromRect;
    Rect toRect;
    int sliderStartX = 0;
    int sliderEndX = 0;

    bool draggingFromRect = false;
    bool draggingToRect = false;

    TimeRange *timeRange = nullptr;
};

}

#endif // __CTL_SLIDER_H__
