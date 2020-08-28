/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Control servos connected to PWM driver onboard of Navio2 shield for Raspberry Pi.

Connect servo to Navio2's rc output and watch it work.
PWM_OUTPUT = 0 complies to channel number 1, 1 to channel number 2 and so on.
To use full range of your servo correct SERVO_MIN and SERVO_MAX according to it's specification.

To run this example navigate to the directory containing it and run following commands:
make
sudo ./Servo
*/

#include <unistd.h>
#include "Navio/PWM.h"
#include "Navio/Util.h"

#include <iostream>
#include <cstdlib>
#include <pthread.h>

using namespace std;



struct arg_struct {
int arg1;
int arg2;
}args;

void *motorcmds(void *arguments)
{
	struct arg_struct *args = (struct arg_struct *)arguments; 

	while(true)
	{
  
		cout << "we inside a thread here (left cmd): " << args->arg1 << endl;
		cout << "we inside a thread here (right cmd): " << args->arg2 << endl;
		sleep(.5);
	}

	pthread_exit(NULL);
}

int main () {

	int rc;
	int winch_left_cmd = 35;
	int winch_right_cmd = -35;
	int i=0;

	pthread_t mcmd;

	struct arg_struct args;
	args.arg1 = winch_left_cmd;
	args.arg2 = winch_right_cmd;

	cout << "main() : creating thread... " << endl;

	rc = pthread_create(&mcmd, NULL, motorcmds, (void *)&args);

	while(i<10)
{
//	struct arg_struct args;
	args.arg1 = winch_left_cmd;
	args.arg2 = winch_right_cmd;

	//cout << "we left the thread" << endl;
	winch_left_cmd++;
	winch_right_cmd--;
	i++;
	cout << "counter (i): " << i << endl;

	sleep(1);

}
	pthread_exit(NULL);

	return 0;

}


