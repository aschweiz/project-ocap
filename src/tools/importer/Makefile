all: ../../common/linalg.h ../../common/splines.h igc_parser.h
#	gcc -o sim main.c -DGL_SILENCE_DEPRECATION -L/System/Library/Frameworks -framework GLUT -framework OpenGL
	g++ -ggdb -lm \
		-o importer \
		-I../../common \
		-I../../libocap \
		../../common/linalg.cpp ../../common/splines.cpp \
		../../libocap/Vector.c \
		main.cpp igc_parser.cpp
