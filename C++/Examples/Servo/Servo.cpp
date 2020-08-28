#include <unistd.h>
#include "Navio/PWM.h"
#include "Navio/Util.h"

#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <unistd.h>
#include <chrono>
#include <atomic>
//#include <mutex>

using namespace std;
using namespace std::chrono;

#define NUM_THREADS 5

int shared_var = 0;

atomic<float> atomic_shared;

auto start = high_resolution_clock::now();

auto stop  = high_resolution_clock::now();

auto duration = duration_cast<microseconds>(stop - start);

//void *PrintHello(void *threadid)
void *PrintHello(void *)
{
	while(true)
	{
		usleep(.01*100000);
		atomic_shared.store(1, memory_order_relaxed);
//		cout << "thread firing" << endl;
	}

	pthread_exit(NULL);

}

int main()
{

	pthread_t threads[1];

	int thr = pthread_create(&threads[1], NULL, PrintHello, NULL);

	if (thr)
	{
		cout << "Error:unable to create thread," << thr << endl;
		exit(-1);
	}

	while(1)
	{

		auto start = high_resolution_clock::now();

		atomic_shared.store(2, memory_order_seq_cst);

		usleep(.001*100000);

		cout << "atomic_shared: " << atomic_shared.load(memory_order_relaxed) << endl;

		auto stop = high_resolution_clock::now();

		auto duration = duration_cast<microseconds>(stop - start);

		cout << "duration: " << duration.count() << endl;

//		cout << "main loop firing" << endl;

	}

}

/*
void *PrintHello(void *threadid)
{
	long tid;
	tid = (long)threadid;
	cout << "Hello World! Thread ID, " << tid << endl;
	pthread_exit(NULL);
}

int main ()
{
	pthread_t threads[NUM_THREADS];
	int rc;
	int i;

	for( i = 0; i < NUM_THREADS; i++ )
	{
		cout << "main() : creating thread, " << i << endl;
		rc = pthread_create(&threads[i], NULL, PrintHello, (void *)i);

		if (rc)
		{
			cout << "Error:unable to create thread," << rc << endl;
			exit(-1);
      		}
	}
	pthread_exit(NULL);
}
*/

