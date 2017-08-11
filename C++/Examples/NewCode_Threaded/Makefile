CXX ?= g++
CFLAGS = -c -Wall
NAVIO = ../../Navio
INCLUDES = -I ../..

all:
	$(CXX) -std=gnu++11 -pthread $(INCLUDES) NewCode.cpp $(NAVIO)/MS5611.cpp $(NAVIO)/I2Cdev.cpp $(NAVIO)/RCInput.cpp $(NAVIO)/MPU9250.cpp $(NAVIO)/LSM9DS1.cpp $(NAVIO)/PWM.cpp $(NAVIO)/ADC.cpp $(NAVIO)/Ublox.cpp  $(NAVIO)/Util.cpp -o NewCode

clean:
	rm NewCode
