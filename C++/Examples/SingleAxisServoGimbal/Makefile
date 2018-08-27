CXX ?= g++
CFLAGS = -c -Wall
NAVIO = ../../Navio
INCLUDES = -I ../..

all:
	$(CXX) -std=gnu++11 $(INCLUDES) SingleAxisServoGimbal.cpp $(NAVIO)/MPU9250.cpp $(NAVIO)/PWM.cpp $(NAVIO)/Util.cpp -o SingleAxisServoGimbal

clean:
	rm SingleAxisServoGimbal
