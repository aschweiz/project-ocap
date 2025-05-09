//
// linalg.h
//
// OCAP - Open Collision Avoidance Protocol
//
// Minimal linear algebra library (just what we need).
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

#ifndef __LINALG_H__
#define __LINALG_H__ 1

#include "Vector.h"
#include <ostream>

namespace linalg {

class Vector3d {
public:
    Vector3d();
    Vector3d(double x, double y, double z);
    Vector3d(Vector3d &other);
    virtual ~Vector3d();

    double X() const;
    double Y() const;
    double Z() const;

    void Set(double x, double y, double z);

    // Assigning by value.
    void operator=(const Vector3d &v);

    // Multiply a vector with a scalar value.
    Vector3d operator*(double v);

    // Subtract a vector.
    Vector3d operator-(Vector3d v);

    // Scalar distance between two vectors.
    double DistanceTo(Vector3d &other) const;

    TVector *GetVector();

private:
    TVector vec;
};

}

std::ostream &operator<<(std::ostream &str, linalg::Vector3d const &v);

linalg::Vector3d operator+(linalg::Vector3d const &a, linalg::Vector3d const &b);

#endif // __LINALG_H__
