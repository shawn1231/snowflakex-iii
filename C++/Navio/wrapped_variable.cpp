#include "wrapped_variable.h"

// useless default constructor
wrapped_variable::wrapped_variable()
{
	original = 0;
	continuous = 0;
	number_of_wraps = 0;
	wrap_size = 0;
	threshold = 0;
}

// useful constructor
wrapped_variable::wrapped_variable(float des_wrap_size, float des_threshold)
{
	original = 0;
	continuous = 0;
	number_of_wraps = 0;
	wrap_size = des_wrap_size;
	threshold = des_threshold;
}

float wrapped_variable::process_new_input(float current_original)
{
	if( current_original > threshold && original <- threshold )
	{
		// value was small, now it is big, offset it in the negative direciton
		number_of_wraps--;
	}
	else if( current_original < -threshold && original > threshold )
	{
		// value was big, now it is small, offset it in the positive direction
		number_of_wraps++;
	}
	else
	{
		// do nothing if we are not at a point of interest
	}
	// update the member variable "original" to reflect the most current value
	original = current_original;
	// update the continous variable using the most current original and the number of wraps
	continuous = current_original + ( number_of_wraps * wrap_size );
	return continuous;
}

float wrapped_variable::get_current_continuous()
{
	// public function offering read access to private member "continous"
	return continuous;
}

float wrapped_variable::get_number_of_wraps()
{
	// public function offering read access to private member "number_of_wraps"
	return number_of_wraps;
}

void wrapped_variable::reset_wrap_counter()
{
	// public function which resets the wrap counter
	number_of_wraps = 0;
	return;
}
