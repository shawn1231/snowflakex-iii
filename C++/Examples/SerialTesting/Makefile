CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../..

all:
	$(CXX) -std=gnu++11 -lwiringPi $(INCLUDES) ADC.cpp $(NAVIO)/Util.cpp $(NAVIO)/ADC.cpp -o ADC

clean:
	rm ADC
