/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi

To run this example navigate to the directory containing it and run following commands:
make
./Barometer
*/

#include "Navio/MS5611.h"
//#include "Navio/Util.h"
#include <unistd.h>
#include <stdio.h>
#include <cmath>

#include <iostream>
#include <fstream>

using namespace std;

int main()
{
    MS5611 barometer;
    float Pb = 29.92126;  //inHg
    float Tb = 288.15; //Kelvin
    float Lb = -.0019812; // K/ft
    float hb = 0; // ft
    float R  = 8.9494596e4; // (lb ft^2)/(lb mol K s^2)
    float g0 = 32.17405; // ft/s^2
    float M  = 28.9644; // lb/lbmol

    float Tc = 0.0; // temp in C
    float Tk = 0.0; // temp in K
    float Tf = 0.0; // temp in F
    float Pm = 0.0;
    float msl = 0.0;

    barometer.initialize();

    float offset[3] = {0,0,0};
    float rotation[3][3] = {{0,0,0},{0,0,0},{0,0,0}};


    ifstream ifs("file.csv");
    if(ifs){
	cout << "The offsets are:" << endl;
	for(int i = 0 ; i < 3 ; i++ ){
		if(i != 0){
			cout << ", ";
			char delim;
			ifs >> delim;}
		ifs >> offset[i];
		cout << offset[i];}
	cout << endl;
	cout << "The rotations are:" << endl;
	for(int i = 0 ; i < 3 ; i++ ){
		for(int j = 0 ; j < 3 ; j++ ){
			if(j != 0){
				cout << ", ";
				char delim;
				ifs >> delim;}
			ifs >> rotation[i][j];
			cout << rotation[i][j];}
		cout << endl;}}









    while (true) {
        barometer.refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer.readPressure();

        barometer.refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer.readTemperature();

        barometer.calculatePressureAndTemperature();

	Tc = barometer.getTemperature();
	Tk = Tc+273.15;
	Tf = Tc*(9.0/5.0)+32;
	Pm = barometer.getPressure()*.02953;
	msl = hb + (Tk/Lb)*(pow((Pm/Pb),((-R*Lb)/(g0*M)))-1);


	printf("Temperature(F)(K)(C):  %f %f %f  Pressure(inHg):  %f  Mean Sea Level Altitutde (ft):  %f\n", Tf,Tk,Tc,Pm,msl);

        sleep(1);
    }

    return 0;
}
