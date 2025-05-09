//
// radio_link.h
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

#ifndef __RADIO_LINK_H__
#define __RADIO_LINK_H__

#include <cinttypes>
#include <map>
#include <string>
#include <vector>

#include "linalg.h"

#define MAX_RADIO_MESSAGE_SIZE_BYTES 64

class IRadioLinkSubscriber;

class RadioMessage {
public:
    RadioMessage();
    RadioMessage(RadioMessage &msg);

public:
    IRadioLinkSubscriber *sender = nullptr;

    long txStartTimeMs = 0;
    int txDurationMs = 0;

    uint8_t bytes[MAX_RADIO_MESSAGE_SIZE_BYTES];
    // linalg::Vector3d pos;
    // linalg::Vector3d vel;
};

class IRadioLinkSubscriber {
public:
    virtual int GetIdNr() = 0;
    virtual bool GetPosition(long ms, linalg::Vector3d *pos) = 0;

    virtual void OnReceiveMessage(RadioMessage &msg, double dbm) = 0;
};

// Simulates the radio link between aircraft by forwarding broadcast
// messages to subscribed receivers.
// Considers attenuation, directional gain, packet collisions.
class RadioLink {
public:
    static RadioLink *Instance();

private:
    RadioLink();

public:
    virtual ~RadioLink();

    // All simulated aircraft equipped with simulated receivers register
    // themselves by subscribing for radio messages.
    void Subscribe(IRadioLinkSubscriber *s);

    // Called by individual aircraft to broadcast their data packet to
    // other aircraft in the neighbourhood.
    void Broadcast(RadioMessage &msg);

    // Called at the end of each second in the simulation.
    // Delivers packets to the subscribers.
    void YieldDistribution(long timeMillisec);

private:
    std::vector<IRadioLinkSubscriber*> subscriptions;
    std::vector<RadioMessage*> messages;
};

#endif // __RADIO_LINK_H__
