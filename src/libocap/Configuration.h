//
// Configuration.h
//
// OCAP - Open Collision Avoidance Protocol
//
// Configuration of this library.
//
// 02.07.2024 ASR  First version.
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

#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__ 1

// Simulation, testing?
#define OCAP_SIMULATION 1


// Maximum number of flight objects that we can track (including our own).
#define FLIGHT_OBJECT_LIST_LENGTH 200

// Maximum number of seconds before we release a flight object.
#define FLIGHT_OBJECT_TIME_TO_RELEASE_SEC 12


// Iteration step in seconds for the prediction algorithm.
// Needs to be fixed at 1 for the current implementation.
#define T_DELTA_SEC 1

// Maximum prediction time in seconds.
#define T_MAX_SEC 30

// Number of seconds back in time for the second tangent vector.
// Increasing this value increases the memory footprint (x 2 vectors).
#define FLIGHT_OBJECT_OWN_EXTRAPOLATION_DEPTH_SEC 5

// Number of seconds during which the Z vector is averaged.
// Increasing this value increases the memory footprint (x 1 vector).
#define FLIGHT_OBJECT_OWN_Z_AVERAGING_SEC 5


// Limit for truncated cone check: 8km (1e8 m2).
// (Corresponds to about 1000km/h, 30s * 1000km/h = 8.3km)
#define DIST_MTR_SQU_CHECK_LIMIT 64000000

// Tolerance (number of own v lengths) for inaccuracies in Z.
// (Caused e.g. by noise on the pos and vel input vectors).
#define ALARM_Z_OWN_COMPENSATION_MILLI_V 1500

// Tolerance (number of other v lengths) for inaccuracies in Z.
// (Caused by the calculation and resolution of the transmitted value.)
#define ALARM_Z_OTHER_COMPENSATION_MILLI_V 2000


// Maximum number of alarm states.
#define ALARM_STATE_LIST_LENGTH 5


#endif // __CONFIGURATION_H__
