#include <sys/time.h>
#include <unistd.h>
#include <iostream>
using namespace std;

struct timeval te;
long long start = 0;
long long end   = 0;
long long length = 10; //seconds
const long long sleepDur = 2000; //useconds
//long long t = 0;

int main()
{
	while(true)
	{
		cout << "Please enter the number of seconds you wish to test\n";
		cin >> length;
		gettimeofday(&te, NULL);
		start = te.tv_sec*1000LL + te.tv_usec/1000;
		cout << "entered the main function\n\n";
		for(int i = 0; i < (length/(sleepDur*1e-6)); i++)
		{
			//gettimeofday(&te, NULL);
			//cout << "shawn";
			//now = te.tv_sec*1000LL + te.tv_usec/1000; //should give time since epoch in ms 
			//cout << now << "\n";
			usleep(sleepDur);
		}

		gettimeofday(&te, NULL);
		end = te.tv_sec*1000LL + te.tv_usec/1000;
		//now = now - offset

		cout << "started at "  << start << " ended at "  << end << "\n";
		cout << "difference " << end-start << " error "  << ((end-start)-length*1e3)  << " us\n";
	}

	return 0;
}
