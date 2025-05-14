//
// ads_l_integration.h
//
// OCAP - Open Collision Avoidance Protocol
//
// Integrating with the ADS-L library.
//
// 12.09.2024 ASR  First version
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

#ifndef __ADS_L_INTEGRATION_H__
#define __ADS_L_INTEGRATION_H__ 1

#include "FlightPathExtrapolation.h"
#include "linalg.h"
#include "radio_link.h"

bool CreateRadioMessage(
    RadioMessage *msg, int addr,
    long txMs, linalg::Vector3d &posNewMtr, linalg::Vector3d &velNewMtrSec,
    linalg::Vector3d *zVecMtr, EOcapPathModel pathModel // optional (for iConspicuity2)
);

bool DecodeRadioMessage(
    RadioMessage &msg,
    linalg::Vector3d &posMtr, linalg::Vector3d &velMtrSec,
    bool *hasZ, linalg::Vector3d &zVecMtr, EOcapPathModel *pathModel);

#endif // __ADS_L_INTEGRATION_H__
