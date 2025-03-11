all: lib
	g++ -ggdb -std=c++17 -lm -o libocap-test main.cpp libocap.a

lib: AlarmState.h AlarmStateList.h Configuration.h FlightObjectOwn.h FlightObjectOther.h FlightObjectState.h FlightObjectList.h FlightObjectOrientation.h Prediction.h Vector.h CalculateOwnData.h CalculateOtherData.h FlightPathExtrapolation.h
	g++ -c -ggdb -std=c++17 -lm -o OcapLogSim.o OcapLogSim.c
	g++ -c -ggdb -std=c++17 -lm -o AlarmStateList.o AlarmStateList.c
	g++ -c -ggdb -std=c++17 -lm -o FlightObjectOwn.o FlightObjectOwn.c
	g++ -c -ggdb -std=c++17 -lm -o FlightObjectOther.o FlightObjectOther.c
	g++ -c -ggdb -std=c++17 -lm -o FlightObjectList.o FlightObjectList.c
	g++ -c -ggdb -std=c++17 -lm -o FlightObjectOrientation.o FlightObjectOrientation.c
	g++ -c -ggdb -std=c++17 -lm -o Prediction.o Prediction.c
	g++ -c -ggdb -std=c++17 -lm -o Vector.o Vector.c
	g++ -c -ggdb -std=c++17 -lm -o CalculateOwnData.o CalculateOwnData.c	
	g++ -c -ggdb -std=c++17 -lm -o CalculateOtherData.o CalculateOtherData.c	
	g++ -c -ggdb -std=c++17 -lm -o FlightPathExtrapolation.o FlightPathExtrapolation.c	
	ar r libocap.a AlarmStateList.o FlightObjectOwn.o FlightObjectOther.o FlightObjectList.o FlightObjectOrientation.o Prediction.o Vector.o CalculateOwnData.o CalculateOtherData.o FlightPathExtrapolation.o OcapLogSim.o
	ranlib libocap.a
	rm *.o

clean:
	rm *.o
	rm libocap.a
	rm libocap-test

