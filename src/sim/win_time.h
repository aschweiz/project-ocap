//
// win_time.h
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

#ifndef __WIN_TIME_H__
#define __WIN_TIME_H__ 1

#include "ctl_slider.h"
#include "test_set.h"
#include "time_range.h"
#include "win_base.h"

namespace gui {

class Win_Time : public Win_Base {
public:
    static Win_Time *Instance();

private:
    Win_Time();

public:
    virtual ~Win_Time();

    void SetTimeRange(TimeRange *tr);

    void SetTestSet(TestSet *ts);

    // Called by GLUT for autoRun.
    virtual void TimerFunc(int value);

protected:
    // Called by base class.
    virtual void CreateSubclass();

    // Overriding base class implementation.
    virtual void Display();
    virtual void Keyboard(unsigned char key, int x, int y);
    virtual void Special(int key, int x, int y);
    virtual void SpecialUp(int key, int x, int y);
    virtual void Mouse(int button, int state, int x, int y);
    virtual void Motion(int x, int y);
    virtual void EnterLeave(int state);

private:
    TestSet *testSet = nullptr;
    CtlSlider *timeSlider = nullptr;
    bool shiftClicked = false;
    int timerCtr = 0;
};

}

#endif // __WIN_TIME_H__
