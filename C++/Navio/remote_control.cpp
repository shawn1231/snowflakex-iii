#include "remote_control.h"

#include <iostream>

// code was previously in the ScaleVars.h file
// has been moved here to try to combine function only files into a single header
float calculate_slope(const float *input_range , const float *output_range)
{
	return (output_range[1] - output_range[0]) / (input_range[1]-input_range[0]);
}

float calculate_intercept(const float *input_range , const float *output_range ,const float &slope)
{
	return (output_range[1] - input_range[1]*slope);
}

float *calculate_coefficients(const float *input_range , const float *output_range)
{
	float slope = calculate_slope(input_range,output_range);
	float intercept = calculate_intercept(input_range,output_range,slope);
	static float coefficients[] = {slope,intercept};
	return coefficients;
}

float scale_output(const float *coefficients , const float &input)
{
	return (input*coefficients[0] + coefficients[1]);
}

remote_control::remote_control()
{
	std::cout << "Initilizing RC Input..................." << std::endl;
	rcinput.init();
	for(int i = 0 ; i < NUM_CHANNELS ; i++ ){
		rc_array[i] = rcinput.read(i);} // read in each value using the private class function (converts to int automatically)
	std::cout << "Currently using " << NUM_CHANNELS << " channels............." << std::endl;
	std::cout << " --RC Input successfully initialized-- " << std::endl;
	std::cout << "Creating RC Input range coefficients   " << std::endl;
	for(int i = 0 ; i < NUM_CHANNELS ; i++){ // using a custom class for converting bounded values to values in a new range
		// in the Python version of this function, both parameters are returned by one function
		// since multiple values may not be returned by a single function this version uses a function for each scaling parameter
		coefficients[i][0] = calculate_slope(input_range,output_range[i]); // first call the slope calculator
		coefficients[i][1] = calculate_intercept(input_range,output_range[i],coefficients[i][0]); // next calculate the intercept
	}
	std::cout << "Successfully created range coefficients" << std::endl;
}

int remote_control::get_raw(int channel)
{
	return rc_array[channel];
}

float remote_control::get_scaled(int channel)
{
	return rc_array_scaled[channel];
}

void remote_control::update()
{
	for(int i = 0 ; i < rcinput.channel_count ; i++ )
	{
		rc_array[i] = rcinput.read(i); // read in each value using the private class function (converts to int automatically)
	}
	for(int i = 0 ; i < rcinput.channel_count ; i++)
	{
		rc_array_scaled[i] = scale_output(coefficients[i],float(rc_array[i]));
	}
}



