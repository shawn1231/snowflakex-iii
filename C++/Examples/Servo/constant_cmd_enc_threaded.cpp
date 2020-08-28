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
 
typedef struct thread_data {
   int cmd1;
   int cmd2;
   int enc1;
   int enc2;

} thread_data;

void *actuators(void *arg)
{
   thread_data *tdata=(thread_data *)arg;

   int cmd1 = 1000;
   int cmd2 = 1000;
   int enc1 = 999;
   int enc2 = 1001;

   tdata->cmd1=cmd1;
   tdata->cmd2=cmd2;
   tdata->enc1=enc1;
   tdata->enc2=enc2;

   pthread_exit(NULL);
}

int main()
{
   pthread_t tid;
   thread_data tdata;

   pthread_create(&tid, NULL, actuators, (void *)&tdata);
   pthread_join(tid, NULL);

   cout << "cmd1: " << tdata.cmd1 << " enc1: " << tdata.enc1 << endl;
   cout << "cmd2: " << tdata.cmd2 << " enc2: " << tdata.enc2 << endl;
   //printf("%d,%d,%d,%d\n", tdata.cmd1, tdata.cmd2, tdata.enc1, tdata.enc2);   

   return 0;
}


