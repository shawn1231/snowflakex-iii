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

#include <string>
#include <unistd.h>
#include "Navio/PWM.h"
#include "Navio/Util.h"

#include <iostream>
#include <cstdlib>
#include <pthread.h>

#include <wiringSerial.h>
#include <wiringPi.h>

using namespace std;

typedef struct thread_data {
	int cmd1;
	int cmd2;
	int enc1;
	int enc2;

} thread_data;

int serial_port;
char dat;


void *actuators(void *arg)
{
	thread_data *tdata=(thread_data *)arg;

	int cmd1 = tdata->cmd1;
	int cmd2 = tdata->cmd2;

	char serial_output;
	serial_output = '+';

	// MOTOR 1

	std::string serialmotor1;
	std::string charcmd1 = std::to_string(cmd1);

	serialmotor1 = "!G 1 ";
	serialmotor1 += charcmd1;
	serialmotor1 += "\r";

	const char * charmotor1 = serialmotor1.c_str();
	serialPuts(serial_port,charmotor1);

	while(serial_output != '\r')
	{
		serial_output = serialGetchar(serial_port);
	}

	serial_output = serialGetchar(serial_port);
	serial_output = serialGetchar(serial_port);

	serialPuts(serial_port,"?F 1\r");

	serial_output = serialGetchar(serial_port);

	while(serial_output != '\r')
	{
		serial_output = serialGetchar(serial_port);
	}

	serial_output = serialGetchar(serial_port);
	serial_output = serialGetchar(serial_port);

	std::string enc1 ("");
	while(serial_output != '\r')
	{
		serial_output = serialGetchar(serial_port);
		enc1 += serial_output;
	}

	enc1.erase(enc1.size()-1);
	int enci1 = stoi(enc1);

	cout << "encoder 1 output: " << enc1;
	cout << "Serial Chars available left (1): " << serialDataAvail(serial_port) << endl;


	serial_output = '+';

	// MOTOR 2

	std::string serialmotor2;
	std::string charcmd2 = std::to_string(cmd2);

	serialmotor2 = "!G 2 ";
	serialmotor2 += charcmd2;
	serialmotor2 += "\r";

	const char * charmotor2 = serialmotor2.c_str();
	serialPuts(serial_port,charmotor2);

	while(serial_output != '\r')
{
	serial_output = serialGetchar(serial_port);
}

	serial_output = serialGetchar(serial_port);
	serial_output = serialGetchar(serial_port);

	serialPuts(serial_port,"?F 2\r");
	serial_output = serialGetchar(serial_port);

	while(serial_output != '\r')
{
	serial_output = serialGetchar(serial_port);
}

	serial_output = serialGetchar(serial_port);
	serial_output = serialGetchar(serial_port);

	std::string enc2 ("");
	while(serial_output != '\r'){
	serial_output = serialGetchar(serial_port);

	enc2 += serial_output;}

	enc2.erase(enc2.size()-1);

	int enci2 = stoi(enc2);

	cout << "encoder 2 output: " << enc2;
	cout << "Serial Chars available right (2): " << serialDataAvail(serial_port) << endl;

	tdata->enc1=enci1;
	tdata->enc2=enci2;

	pthread_exit(NULL);

}

int main()
{


	cout << "Opening serial port.........." << endl;
	serial_port = serialOpen("/dev/ttyAMA0",115200);
	wiringPiSetup() == -1;	
	cout << "     --Serial UART enabled--     " << endl;

	pthread_t tid;
	thread_data tdata;

	int winch_left_cmd = 0;
	int winch_right_cmd = 0;

	tdata.cmd1 = winch_left_cmd;
	tdata.cmd2 = winch_right_cmd;

	pthread_create(&tid, NULL, actuators, (void *)&tdata);
	pthread_join(tid, NULL);

	cout << "cmd1: " << tdata.cmd1 << " enc1: " << tdata.enc1 << endl;
	cout << "cmd2: " << tdata.cmd2 << " enc2: " << tdata.enc2 << endl;


	return 0;
}


