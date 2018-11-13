#include "vehicle.h"

#define PWMFREQ 50
#define NUM_MOTORS 2

vehicle::vehicle()
{

	std::cout << "Initializing PWM Output................" << std::endl;
	for( int i = 0 ; i < NUM_MOTORS ; i++ )
	{
		if(!pwm_out.init(i))
		{
			std::cout << "Cannot initialize motor: " << i << std::endl;
		}
	}
	std::cout << "Enabling PWM Output Channels..........." << std::endl;
	std::cout << "Setting PWM Period for " << PWMFREQ << "Hz............" << std::endl;
	for( int i = 0 ; i < NUM_MOTORS ; i++ )
	{
		pwm_out.enable(i);
		pwm_out.set_period(i,PWMFREQ);
	}
}


