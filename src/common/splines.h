//
// splines.h
//
// OCAP - Open Collision Avoidance Protocol
//
// Minimal splines implementation.
// We use splines to map continuous flight paths in the simulation.
//
// 26.03.2024 ASR  First version
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

#ifndef __SPLINES_H__
#define __SPLINES_H__ 1

#include <ostream>
#include <vector>

#include "linalg.h"

namespace splines {

class Spline_CentripetalCatmullRom {
public:
    Spline_CentripetalCatmullRom();
    virtual ~Spline_CentripetalCatmullRom();

    size_t Size() const;

    // Fill the spline with points.
    void AppendPointAtTime(linalg::Vector3d &p, double timeSec);

    // Calculate the interpolation at a specific point in time.
    linalg::Vector3d *InterpolatePointAtTime(double timeSec);

private:
    std::vector<linalg::Vector3d*> p_i;
    std::vector<double> timeSec_i;
    std::vector<double> t_i;
};

}

std::ostream &operator<<(std::ostream &str, splines::Spline_CentripetalCatmullRom const &v);

#endif // __SPLINES_H__
