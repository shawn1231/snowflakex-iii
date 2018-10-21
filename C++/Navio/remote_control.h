#ifndef REMOTECONTROLH_
#define REMOTECONTROLH_

#include "Navio/RCInput.h"

#define PWM_MIN 1088
#define PWM_MAX 1920

//class RCInput;

class remote_control
{
	private:
		RCInput rcinput{};
		const float input_range[2] = {PWM_MIN,PWM_MAX}; // range is the same for all channels
		const float output_range[NUM_CHANNELS][2] = {{-.05,.30},{2,-2},{-.185,.500},{-180,180},{-.1,.1},{-.1,.1}};
		float coefficients[NUM_CHANNELS][2];
		int rc_array[NUM_CHANNELS]; // array for holding the values which are read from the controller
		float rc_array_scaled[NUM_CHANNELS]; // array for holding the scaled values
		
	public:
		remote_control();
		int get_raw(int);
		float get_scaled(int);
		void update();
};


#endif