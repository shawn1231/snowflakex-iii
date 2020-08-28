#ifndef SCHEDULERH_
#define SCHEDULERH_

#include <sys/time.h>
#include <cstddef>
#include <iostream>

#define NUM_LOOPS  9

class scheduler
{
	private:
		struct timeval timeobj;
		unsigned long long tse;
		unsigned long long time_start;
		unsigned long long time_now;
		const float frequency [NUM_LOOPS] = {50,1000,5,50,50,20,10,.1,1};//Hz
		unsigned long long duration [NUM_LOOPS]; // stores the expected time since last execution for a given loop
		unsigned long long timer [NUM_LOOPS]; // stores the time since the last execution in a given loop
		unsigned long long watcher [NUM_LOOPS]; // used for monitoring actual timer loop durations

	public:
		scheduler();
		void update_time();
		unsigned long long get_current_time();
		unsigned long long get_duration(int);
		unsigned long long get_frequency(int);
		unsigned long long get_timer(int);
		unsigned long long get_watcher(int);
		int update_watcher(int);
		int update_timer(int);
};


#endif
