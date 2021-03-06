#ifndef VEHICLEH_
#define VEHICLEH_

#include <sys/time.h>
#include <cstddef>
#include <iostream>
#include "scheduler.h"
#include "barometer.h"
#include "remote_control.h"
#include "Navio/PWM.h"
//#include "Navio/RCInput.h"

#define NUM_LOOPS  9

class vehicle
{
	public:
		class scheduler timer;
		class barometer baro;
		PWM pwm_out;
		class remote_control rc;
		vehicle();
	// private:
		// struct timeval timeobj;
		// unsigned long long tse;
		// unsigned long long time_start;
		// unsigned long long time_now;
		// const float frequency [NUM_LOOPS] = {300,1000,5,100,50,20,10,.1,1};//Hz
		// unsigned long long duration [NUM_LOOPS]; // stores the expected time since last execution for a given loop
		// unsigned long long timer [NUM_LOOPS]; // stores the time since the last execution in a given loop
		// unsigned long long watcher [NUM_LOOPS]; // used for monitoring actual timer loop durations
		
	// public:
		// vehicle();
		// void update_time();
		// unsigned long long get_current_time();
		// unsigned long long get_duration(int);
		// unsigned long long get_frequency(int);
		// unsigned long long get_timer(int);
		// unsigned long long get_watcher(int);
		// int update_watcher(int);
		// int update_timer(int);
};


#endif