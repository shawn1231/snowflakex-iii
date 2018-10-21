CXX ?= g++
CFLAGS = -c -Wall
NAVIO = ../../Navio
INCLUDES = -I ../..

all:
	$(CXX) -std=gnu++11 -lwiringPi $(INCLUDES) NewCode.cpp $(NAVIO)/MS5611.cpp $(NAVIO)/I2Cdev.cpp $(NAVIO)/MPU9250.cpp $(NAVIO)/LSM9DS1.cpp $(NAVIO)/PWM.cpp $(NAVIO)/RCInput.cpp  $(NAVIO)/ADC.cpp $(NAVIO)/wrapped_variable.cpp $(NAVIO)/data_file.cpp $(NAVIO)/testing.h $(NAVIO)/vehicle.cpp $(NAVIO)/scheduler.cpp $(NAVIO)/barometer.cpp $(NAVIO)/remote_control.cpp $(NAVIO)/Ublox.cpp  $(NAVIO)/Util.cpp -o NewCode 

clean:
	rm NewCode
