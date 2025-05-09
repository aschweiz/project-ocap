//
// splines.cpp
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

#include <iostream>
#include <cmath>
#include "splines.h"

using namespace linalg;
using namespace splines;

Spline_CentripetalCatmullRom::Spline_CentripetalCatmullRom()
{
}

Spline_CentripetalCatmullRom::~Spline_CentripetalCatmullRom()
{
}

size_t Spline_CentripetalCatmullRom::Size() const
{
	return p_i.size();
}

void Spline_CentripetalCatmullRom::AppendPointAtTime(linalg::Vector3d &p, double timeSec)
{
	double tNew = 0;

	if (p_i.size() > 0) {
		auto pOld = p_i.back();
		auto tOld = t_i.back();
		tNew = tOld + pow(p.DistanceTo(*pOld), 0.0);
	}

	linalg::Vector3d *pNew = new linalg::Vector3d(p);
	p_i.push_back(pNew);
	t_i.push_back(tNew);
	timeSec_i.push_back(timeSec);
}

linalg::Vector3d *Spline_CentripetalCatmullRom::InterpolatePointAtTime(double timeSec)
{
	// The algorithm uses points p0, p1, p2, p3 for the segment p1 to p2.
	// We first need to find out which points to use.
	int i2 = -1;
	for (int i = 0, imax = (int)timeSec_i.size(); i < imax; i++) {
		auto tSec = timeSec_i[i];
		if (tSec > timeSec) {
			i2 = i;
			break;
		}
	}
	// After the end of the track?
	int i3 = i2 + 1;
	if (i2 == -1 || i3 >= p_i.size()) {
		return p_i[p_i.size() - 2];
	}
	// Before the beginning of the track?
	int i1 = i2 - 1;
	int i0 = i2 - 2;
	if (i1 < 0 || i0 < 0) {
		return p_i[1];
	}

	// We have indexes i0..i3.
	// Interpolate offset between points (v in 0..1).
	double timeSec1 = timeSec_i[i1];
	double timeSec2 = timeSec_i[i2];
	double v = (timeSec - timeSec1) / (timeSec2 - timeSec1);

	// We have time interpolation v in 0..1 between indexes 1 and 2.
	// Get the corresponding t value.
	double t0 = t_i[i0];
	double t1 = t_i[i1];
	double t2 = t_i[i2];
	double t3 = t_i[i3];
	double t = t1 + v * (t2 - t1);

	// Get the vectors involved in the calculation.
	Vector3d &p0 = *p_i[i0];
	Vector3d &p1 = *p_i[i1];
	Vector3d &p2 = *p_i[i2];
	Vector3d &p3 = *p_i[i3];

	// Calculate the auxilary vectors.
	Vector3d a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
	Vector3d a2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1));
	Vector3d a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));
	Vector3d b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0));
	Vector3d b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1));

	// Calculate the result vector.
	Vector3d c  = b1 * ((t2 - t) / (t2 - t1)) + b2 * ((t - t1) / (t2 - t1));
	return new Vector3d(c);
}

std::ostream &operator<<(std::ostream &str, Spline_CentripetalCatmullRom const &v)
{
	str << "Spline(len=" << v.Size() << ")";
	return str;
}
