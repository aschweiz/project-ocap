//
// radio_link.cpp
//
// OCAP - Open Collision Avoidance Protocol
//
// Simulates the radio link between transmitting and receiving aircraft.
// We use a publish-subscribe mechanism to distribute the broadcast messages
// from the transmitter to all subscribed receivers.
//
// 03.07.2024 ASR  First version
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

#include "radio_link.h"
#include "lon_lat_util.h"

static RadioLink *sInstance;

RadioMessage::RadioMessage()
{
}

RadioMessage::RadioMessage(RadioMessage &msg)
{
    sender = msg.sender;
    txStartTimeMs = msg.txStartTimeMs;
    txDurationMs = msg.txDurationMs;
    for (int i = 0, imax = sizeof(bytes) / sizeof(uint8_t); i < imax; i++) {
        bytes[i] = msg.bytes[i];
    }
}

RadioLink *RadioLink::Instance()
{
    if (!sInstance) {
        sInstance = new RadioLink();
    }
    return sInstance;
}

RadioLink::RadioLink()
{
}

RadioLink::~RadioLink()
{
}

void RadioLink::Subscribe(IRadioLinkSubscriber *s)
{
    subscriptions.push_back(s);
}

void RadioLink::Broadcast(RadioMessage &msg)
{
    // Called by individual aircraft to broadcast their data packet to
    // other aircraft in the neighbourhood.
    RadioMessage *m = new RadioMessage(msg);
    messages.push_back(m);
}

void RadioLink::YieldDistribution(long timeMillisec)
{
    printf("----- YieldDistribution %ld\n", timeMillisec);

    // Send the messages to the subscribers.
    for (auto itSubs = subscriptions.begin(); itSubs != subscriptions.end(); itSubs++) {
        for (auto itMsg = messages.begin(); itMsg != messages.end(); itMsg++) {
            auto receiver = *itSubs;
            auto message = *itMsg;

            // A transmitter doesn't receive its transmitted messages.
            if (receiver == message->sender) {
                continue;
            }

            // Sender and receiver position need to be known to calculate the signal
            // strength at the receiver.
            linalg::Vector3d senderPositionDeg;
            bool hasSenderPos = message->sender->GetPosition(timeMillisec, &senderPositionDeg);
            linalg::Vector3d receiverPositionDeg;
            bool hasReceiverPos = receiver->GetPosition(timeMillisec, &receiverPositionDeg);
            if (!hasSenderPos || !hasReceiverPos) {
                continue;
            }

            linalg::Vector3d senderPositionMtr;
            lonDegLatDegAltMtrToMtrMtrMtr(senderPositionDeg.GetVector(), senderPositionMtr.GetVector());
            linalg::Vector3d receiverPositionMtr;
            lonDegLatDegAltMtrToMtrMtrMtr(receiverPositionDeg.GetVector(), receiverPositionMtr.GetVector());
            double distM = receiverPositionMtr.DistanceTo(senderPositionMtr);

            // Attenuation according to distance, orientation / directionality etc.

            double txPwrDbm = 14; // Permitted power on 868.2 MHz: 14 dBm = 25 mW
            double rxSensitivity = -110; // Depends on the receiver and baud rate.
            double systemLoss = 3; // Circuit, antenna etc.
            double fsplConst = 31.2; // Free-space frequency (868.2 MHz) and speed-of-light terms.

            double freeSpacePathLossDb = 20 * log10(distM) + fsplConst;
            double rxDbm = txPwrDbm - systemLoss - freeSpacePathLossDb;
            bool shouldReceive = rxDbm >= rxSensitivity;

            printf("RX_SIG at receiver %d : Distance (km) = %5.1f, Signal (dBm) = %+5.1f (%s)\n",
                receiver->GetIdNr(),
                distM / 1000, rxDbm, shouldReceive ? "Keep" : "Drop");

            // Drop signals if they are too weak.
            if (!shouldReceive) {
                continue;
            }

            (*itSubs)->OnReceiveMessage(**itMsg, -30);
        }
    }

    // Release the messages.
    for (auto itMsg = messages.begin(); itMsg != messages.end(); itMsg++) {
        delete *itMsg;
    }
    messages.clear();
}
