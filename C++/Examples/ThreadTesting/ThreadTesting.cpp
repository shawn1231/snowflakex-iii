#include <unistd.h>
#include <iostream>
#include <time.h>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace std;

mutex mtx;
condition_variable cv;

int number = 0;
bool buffer_full = false;
bool buffer_empty = false;
int buffer;

void gps_thread(){
	while(true){
		number = rand() % 10 + 1;
//		cout << number << endl;
		sleep(1);
		if(number < 7){ // GPS true condition
			cout << "Producer loop says gps data is ready.  ";
			buffer_empty = false;
			mtx.lock();
			buffer = number;
			mtx.unlock();
			buffer_full = true;
		}
	}
}




int main(){

	srand(time(NULL));

	thread t1(gps_thread);


	while(true){

		if(buffer_full){
			buffer_full = false;
			mtx.lock();
			cout << "Consumer loop is using the data.  The data is:  " << number << endl;
			mtx.unlock();
			buffer_empty = true;
		}
		else{
			cout << "This loops happens regularly (250ms)." << endl;
			usleep(250000);
		}

	}


	t1.join();

	return 0;
}
