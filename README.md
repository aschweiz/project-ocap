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

## Getting started

We recommend getting the simulation environment and the automated tests up and running 
first. The easiest way to do this is to use a Linux environment (e.g. Ubuntu 24.04) with 
developer tools installed and carry out the following steps:

```bash
# 1. Compile the libadsl static library
pushd src/libadsl
make
popd
# 2. Compile the libocap static library
pushd src/libocap
make
popd
# 3. Compile the simulation environment
pushd src/sim
make
popd
# 4. Run the automated regression tests
make
```

The last "make" executes all test cases declared in the outermost Makefile. The result
of running the tests is written in the output directory, one file for each test case.
Ideally, if you haven't changed the source code, the result files should be identical 
to the reference output files in the repository.

Once the simulation environment is up and running, have a look at how libadsl and libocap
are integrated in the simulator, as an example of how to integrate these libraries into
custom software.

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

Part of this project is a simulation environment.

During the development phase of the algorithm, the simulation environment was used to check and optimize the algorithm. For further development, regression tests can be carried out using the recorded test cases.

The simulation environment is started with a test case file (.tst) as input. This file describes a single test case consisting of a set of flight objects. For each flight object, a flight path file (.flp) is referenced from the test case, which describes the exact position of the flight object at every second. The "Flight path importer" utility program (see description below) is used to create flight path files.

### Compiling the simulation environment

The source code of the simulation environment is provided in the src/sim directory. First, compile the libadsl and libocap libraries by invoking the make command in the corresponding directories. Afterwards, you can compile the simulator by invoking make in the src/sim directory. The result is the sim binary.

### Starting the simulation environment

It is possible to run the simulation environment in an online session with a GUI or from the command line for automated testing.

To start an online session, invoke the sim binary with a test case file as input, for example:

$ ./src/sim/sim testflights/20250408_grenchen/test_grenchen1.tst

This will start the simulation environment and load the specified test case, including the flight pathes of all aircraft defined in the test case.

### Using the simulation environment

The GUI of the simulation environment consists of 3 windows.

The main window is a 3-d representation of the simulated world. At the top left, you see the loaded test case and the time offset into the simulation, in seconds. All loaded aircraft appear in the view, annotated with their name. If the window has the focus, you can use the cursor keys to rotate (or move, if the shift key is pressed) the view, and the + and - keys to zoom in or out. 

The simulation time window below the main window lets you set the simulation time with the slider control. Drag the slider buttons with the mouse to set the start and end of the displayed flight path for each aircraft in the main window. After selecting a "main" aircraft in the object window, you can press the "space" key to start and stop the collision detection calculation. Use the right arrow cursor to advance the time by 0.1 (or 1.0 with shift pressed) seconds.

The object window at the right side of the screen contains a list of loaded aircraft. Each list entry contains the identifier of the aircraft as well as the ground speed, bearing and vertical speed, on the first line. If the collision prediction is running, the second line contains the distance to the main aircraft, in meters, at the current point in time, and the third line contains information about a potential collision warning alarm message caused by the corresponding aircraft. Alarm messages range from level 1 (lowest) to 3 (most critical). 

## Support tools

### Flight path importer

The Flight Path Importer is used to create flight path files (.flp) based on recorded IGC files. The tool performs spline interpolation to obtain smoothed positions at every second.

