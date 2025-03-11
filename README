# Project OCAP

## Introduction

Project OCAP provides an open-source implementation of an open collision warning algorithm 
for aircraft. All program code is licensed with a BSD license and may be integrated into
vendor software without royalty.

This is the source code repository for Project OCAP. You may want to visit the project 
web site: https://project-ocap.net

The algorithm is independent from the underlying radio protocol. To calculate collision 
warnings, it needs information (such as position, velocity, flight direction etc.)
about the own and surrounding aircraft. Information about the own aircraft needs to be 
generated on board, typically by using an off-the-shelf navigation satellite module. 
Information about surrounding aircraft comes from received radio packets from surrounding 
aircraft. The source code for the collision warning algorithm is provided as a program
library **libocap** in the src/libocap directory.

Project OCAP includes an implementation to receive and transmit radio packets compliant 
with the EASA ADS-L protocol. The source code for the ADS-L implementation is provided
as a program library **libadsl** in the src/libadsl directory.

## libocap

TODO

## libadsl

### Transmitting packets

To transmit an ADS-L packet, you need to provide up-to-date data about the own aircraft 
in two C structures, `SGpsData` and `SAircraftConfig`. Call `createAdslPacket` with the
two structures as input to create an ADS-L packet, and create the bytes to transmit via
`adslEncodeIConspicuity'.

### Receiving packets

To receive an ADS-L packet, you pass the received data bytes to the function 
`adslDecodeIConspicuity` which extracts an ADS-L packet from the data. You can then
extract an `SGpsData`and `SAircraftConfig`, representing information about the received
aircraft, from this packet.

### Test case

The source code directory contains a test case in main.cpp which you can use to test the
code and see how the above functions are used.


