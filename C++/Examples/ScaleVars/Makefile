CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../..

all:
	$(CXX) -std=gnu++11  $(INCLUDES)  $(NAVIO)/LSM9DS1.cpp $(NAVIO)/MPU9250.cpp $(NAVIO)/Util.cpp ScaleVars.cpp -o ScaleVars

clean:
	rm -f AccelGyroMag
