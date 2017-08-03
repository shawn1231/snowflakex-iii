/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Read accelerometer, gyroscope and magnetometer values from
inertial measurement unit: MPU9250 or LSM9DS1 over SPI on Raspberry Pi + Navio.

Navio's onboard sensors are connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1 (MPU9250), /dev/spidev0.3 (acc/gyro LSM9DS1)
and /dev/spidev0.2 (mag LSM9DS1).
*/


#include "Navio/ScaleVars.h"
#include <iostream>
//#include <sys/time.h>
#define NUM_STEPS 21


using namespace std;

int main()
{

	float inputs[NUM_STEPS];
	const float input_range[2] = {1,3};
	const float output_range[2] = {200,-100};
	float step_size = (input_range[1] - input_range[0])/(NUM_STEPS-1);

	for(int i = 0 ; i < NUM_STEPS ; i++)
	{
		if(i == 0)
		{
			inputs[i] = input_range[0];
		}
		else
		{
			inputs[i] = step_size + inputs[i-1];
		}
		cout << inputs[i] << ", ";
	}
	cout << endl;


	cout << input_range[0] << ", " << input_range[1] << endl;
	cout << output_range[0] << ", " << output_range[1] << endl;


	float slope = calculate_slope(input_range, output_range);
	float intercept = calculate_intercept(input_range, output_range, slope);

	float *coeff = calculate_coefficients(input_range,output_range);




//	cout << slope << ", " << intercept << endl;
//	cout << test[0] << ", " << test[1] << endl;

	cout << coeff[0] << ", " << coeff[1] << endl;

//	float *coefficients;
//	coefficients = calculate_coefficients(input_range,output_range);

//	cout << coefficients[02] << coefficients[1] << endl;

	for(int i = 0 ; i < NUM_STEPS ; i ++)
	{
		cout << scale_output(coeff, inputs[i]) << ", ";
	}
	cout << endl;

	int counter = 0;
	float cmd = 0;
	float step = 0;
	while(true)
	{
		if(counter > 100000 && counter < 200000)
		{
			step = 1;
		} if(counter > 200000 && counter < 300000)
		{
			step = 2;
		} if(counter > 300000 && counter < 400000)
		{
			step = 3;
		} if(counter > 400000)
		{
			counter = 0;
		}
		cmd = .5+step*.25;
		//sleep(1);
		cout << cmd << endl;
		counter++;
	}

    return 0;
}
