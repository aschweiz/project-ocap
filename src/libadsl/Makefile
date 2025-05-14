all: lib
	g++ -ggdb -std=c++17 -lm -o libadsl-test main.cpp libadsl.a

lib: ads_l_crc.h ads_l_decode_iconspicuity.h ads_l_encode_iconspicuity.h \
	ads_l_packet.h ads_l_packet_iconspicuity.h \
	ads_l_xxtea.h create_ads_l_packet.h decode_ads_l_packet.h
	g++ -c -ggdb -std=c++17 -lm -o ads_l_crc.o ads_l_crc.c
	g++ -c -ggdb -std=c++17 -lm -o ads_l_decode_iconspicuity.o ads_l_decode_iconspicuity.c
	g++ -c -ggdb -std=c++17 -lm -o ads_l_encode_iconspicuity.o ads_l_encode_iconspicuity.c
	g++ -c -ggdb -std=c++17 -lm -o ads_l_xxtea.o ads_l_xxtea.c
	g++ -c -ggdb -std=c++17 -lm -o create_ads_l_packet.o create_ads_l_packet.c
	g++ -c -ggdb -std=c++17 -lm -o decode_ads_l_packet.o decode_ads_l_packet.c
	ar r libadsl.a ads_l_crc.o ads_l_decode_iconspicuity.o ads_l_encode_iconspicuity.o \
		ads_l_xxtea.o create_ads_l_packet.o decode_ads_l_packet.o
	ranlib libadsl.a
	rm *.o

clean:
	rm *.o
	rm libadsl.a
	rm libadsl-test

