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