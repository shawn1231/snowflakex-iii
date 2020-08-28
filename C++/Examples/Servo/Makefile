CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../..

all:
	$(CXX) -std=gnu++11 -lwiringPi -lpthread $(INCLUDES) Servo.cpp $(NAVIO)/PWM.cpp $(NAVIO)/Util.cpp -o Servo

clean:
	rm Servo
