#ifndef _TESTINGH_
#define _TESTINGH_

#include <stdio.h>
#include <math.h>

void scale_pulse_freq(int *pulse_state, int *counter, const int counter_max)
{

	if(*counter >= counter_max)
	{
		// fire off a pulse
		*pulse_state = 1;
		// set the counter back to zero
		*counter = 0;
	}
	else
	{
		*pulse_state = 0;
		*counter = *counter + 1;
	}
	
	return;
}

int flip_sign(int current_state)
{
	int new_state = 0;
	if(current_state != 1) // catches non standard cases other than current_state = -1 which we are expecting
	{
		new_state = 1;
	}
	else
	{
		new_state = -1;
	}
	// return = 0 is an error for this function
	return new_state;
}
	
int flip_state(int current_state)
{
	int new_state = -1;
	if(current_state != 1)
	{
		new_state = 1;
	}
	else
	{
		new_state = 0;
	}
	// return = -1 is an error for this function
	return new_state;
}

float maneuver_generator(const int step, const int plus_minus, int multi_step, const float des_period, const float des_amp, const int num_steps, const float des_delay)
{
	
	float out = 0;
	
	if(!step)
	{
		multi_step = false;
	}
	
	static int pulse_1 = 0;
	static int pulse_2 = 0;
	static int counter_1 = -1;
	static int step_num = -1;
	static int state_1 = 1;
	static int state_2 = 1;
	static int state_3 = -1;
	static int delay_counter = 1;
	
	const float Ts = .2;
	
	const float period_1 = des_period-Ts;	
	const float counter_max_1 = round(period_1/Ts);
	const float step_size = des_amp/num_steps;
	const int delay_act = (round(des_delay/Ts)+2);
	const int num_steps_act = num_steps - 1;
		
	scale_pulse_freq(&pulse_1,&counter_1,counter_max_1);
	
	if(!(delay_counter < delay_act))
	{
		
		int state_1_prev = state_1;
		
		if(pulse_1)
		{
			if(!multi_step)
			{
				if(abs(step_num) > num_steps_act)
				{
					state_3 = flip_sign(state_3);
				}
			} else
			{
				if(state_1)
				{
					if(abs(step_num) > num_steps_act)
					{
						state_3 = flip_sign(state_3);
					}
					if(!step_num)
					{
						state_2 = flip_sign(state_2);
					}
				}
			}
			state_1 = flip_state(state_1);
			if(!multi_step)
			{
				step_num = step_num + state_3;
			} else
			{
				if(state_1)
				{
					step_num = step_num + state_3;
				}
			}
		}
		pulse_2 = (state_1 && !state_1_prev);
		
		if(!multi_step)
		{
			if(pulse_2)
			{
				state_2 = flip_sign(state_2);
			}
		}
	
		if(step)
		{
			if(!plus_minus)
			{
				//if we don't want pos/neg then just clamp the sign state to positive
				state_2 = 1;
			}
			
			if(!multi_step)
			{
					out = state_1*state_2*des_amp;
			} else
			{
				out = state_1*state_2*abs(step_num)*step_size;
			}
		} else
		{
			out = -step_num*step_size;
		}
	} else
	{
		delay_counter = delay_counter + 1;
		out = 0;
	}
	
	return out;
}

#endif