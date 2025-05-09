//
// linalg.cpp
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

#include "linalg.h"
#include <cmath>

using namespace linalg;

Vector3d::Vector3d()
{
    vectorClear(&vec);
}

Vector3d::Vector3d(double x, double y, double z)
{
    vectorInit(&vec, x, y, z);
}

Vector3d::Vector3d(Vector3d &other)
{
    vectorInit(&vec, other.X(), other.Y(), other.Z());
}

Vector3d::~Vector3d()
{
}

double Vector3d::X() const
{
    return vec.x;
}

double Vector3d::Y() const
{
    return vec.y;
}

double Vector3d::Z() const
{
    return vec.z;
}

void Vector3d::Set(double x1, double y1, double z1)
{
    vectorInit(&vec, x1, y1, z1);
}

void Vector3d::operator=(const Vector3d &v)
{
    vectorInit(&vec, v.X(), v.Y(), v.Z());
}

Vector3d Vector3d::operator*(double v)
{
    Vector3d vec(X() * v, Y() * v, Z() * v);
    return vec;
}

Vector3d Vector3d::operator-(Vector3d v)
{
    Vector3d vec(X() - v.X(), Y() - v.Y(), Z() - v.Z());
    return vec;
}

double Vector3d::DistanceTo(Vector3d &other) const
{
    double dX = X() - other.X();
    double dY = Y() - other.Y();
    double dZ = Z() - other.Z();

    double d2 = dX * dX + dY * dY + dZ * dZ;
    return sqrt(d2);
}

TVector *Vector3d::GetVector()
{
    return &vec;
}

std::ostream &operator<<(std::ostream &str, Vector3d const &v)
{
    str << "(" << v.X() << ", " << v.Y() << ", " << v.Z() << ")";
    return str;
}

Vector3d operator+(Vector3d const &a, Vector3d const &b)
{
    Vector3d c(a.X() + b.X(), a.Y() + b.Y(), a.Z() + b.Z());
    return c;
}
