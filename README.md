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

## libocap

### Configuration

Configurable parameters are stored in the file `Configuration.h`. Make sure to set the
constant `OCAP_SIMULATION` to 0 when integrating the library into vendor software. Other
constants such as `FLIGHT_OBJECT_LIST_LENGTH` may need to be modified depending on 
available memory etc.

### Logging

To support integration testing, the OCAP library includes a logging facility. Implement
the functions declared in `OcapLog.h` for this purpose. An example implementation is 
provided in `OcapLogSim.c`. Provide empty methods if logging isn't required.

### Providing data about own aircraft

Call the method `calculateOwnDataFromGpsInfo` from `CalculateOwnData.h` once per second.
Pass the current time, position and velocity vector into the method.

### Providing data about other aircraft

Call the method `calculateOtherDataFromInfo` from `CalculateOtherData.h` after receiving
information from surrounding aircraft, for example after decoding a received ADS-L packet
with `adslDecodeIConspicuity`. The method takes a pointer to an aircraft
(`TFlightObjectOther`) as well as a position and velocity vector for the aircraft as 
input parameters. Optionally, information about the curvature of the flight path can
be provided, as proposed in our ADS-L extension request (see
https://www.project-ocap.net/post/proposed-ads-l-extension).

The OCAP library maintains a small database of surrounding aircraft. The functions to
manage this database are provided in `FlightObjectList.h`. Aircraft are identified by
a unique ID. Call `flightObjectListGetOther` to search for an aircraft with a specific
ID in the database. Call `flightObjectListAddOther` after receiving an aircraft that's
not yet in the database. Device vendors may replace the aircraft database with their
own implementation or merge the implementations to save memory.

### Calculating collision warnings

Call `predictionInit` once at the beginning to configure and initialize the
prediction logic.

At the end of every second, after the own data and data of surrounding aircraft have
been provided to the library, call `predictionCalculateAlarmStates` in 
`Prediction.h`. For each potential collision that the method detects, it fills 
an alarm entry in the alarm state list. 

After calling the prediction method, 
you can check how many alarms have been generated by calling `alarmStateListGetCount`
and retrieve individual alarm entries with `alarmStateListGetAtIndex` in
`AlarmStateList.h`. Alarm state entries contain an alarm level (from 1 = low to
 3 = high), time in seconds to encounter and a pointer to the affected aircraft.
 
 If needed, you can calculate the orientation of the affected aircraft relative
 to the own aircraft with `flightObjectOrientationCalculate` 
 in `FlightObjectOrientation.h`.
 
 ### Test case

The source code directory contains a test case in main.cpp which you can use to test the
code and see how the above functions are used.

## Simulation environment

TODO

## Support tools

TODO


