#include "scheduler.h"

scheduler::scheduler()
{
	std::cout << "Initializing Time Objects.............." << std::endl;
	gettimeofday(&timeobj, NULL);
	tse = 0;
	time_now = 0;
	std::cout << "Establishing loop frequencies.........." << std::endl;
	//frequency [NUM_LOOPS] = {300,1000,5,100,50,20,10,.1,1}; // .1 = loop_update_hz
	std::cout << "Calculating loop delay durations......." << std::endl;
	std::cout << "Populating loop timers................." << std::endl;
	for(int i = 0 ; i < NUM_LOOPS ; i++){ // populate the durations using the passed values for frequency, echo everything to the console
		duration[i] = 1000000/frequency[i];
		timer[i] = 0;
		watcher[i] = 0;}
}

void scheduler::update_time()
{
	// refresh time now to prepare for another loop execution
	gettimeofday(&timeobj, NULL); // must first update the time_obj
	tse        = timeobj.tv_sec*1000000LL + timeobj.tv_usec; // update tse (us)
	//tse        = tse + 2000000000; // uncomment to test integer overflow fix
	time_now   = tse - time_start; // calculate the time since execution start by subtracting off tse
}

unsigned long long scheduler::get_current_time()
{
	return time_now;
}

unsigned long long scheduler::get_duration(int identifier)
{
	if( identifier > NUM_LOOPS )
	{
		//invalid identifier
		return 1;
	}else
	{
		return duration[identifier];
	}
}

unsigned long long scheduler::get_frequency(int identifier)
{
	if( identifier > NUM_LOOPS )
	{
		//invalid identifier
		return 1;
	}else
	{
		return frequency[identifier];
	}
}

unsigned long long scheduler::get_timer(int identifier)
{
	if( identifier > NUM_LOOPS )
	{
		//invalid identifier
		return 1;
	}else
	{
		return timer[identifier];
	}
}

unsigned long long scheduler::get_watcher(int identifier)
{
	if( identifier > NUM_LOOPS )
	{
		//invalid identifier
		return 1;
	}else
	{
		return watcher[identifier];
	}
}

int scheduler::update_watcher(int identifier)
{
	if( identifier > NUM_LOOPS )
	{
		//invalid identifier
		return 1;
	}else
	{
	watcher[identifier] = time_now - timer[identifier];
	return -0;
	}
}

int scheduler::update_timer(int identifier)
{
	if( identifier > NUM_LOOPS )
	{
		return 1;
	}else
	{
		timer[identifier] = time_now;
		return 0;
	}
}
