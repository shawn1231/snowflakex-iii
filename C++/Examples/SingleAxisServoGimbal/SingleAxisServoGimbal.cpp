//---------------------------------------------------------------------------------------
// Author:	Shawn Herrington
// Date:	01/11/2018
// Purpose:	Control a single axis servo gimbal using Navio2 state estimate from
//		Madgwick filter
// Notes:	Proof of concept TA solution to the first Lab for ME457 Spring 2018
// Notes:	Combining example code provided by Emlid for Servo command with example
//		code provided for AHRS (euler orientatino angles)
// Notes:	Set Madgwick filter gain (beta) in file "AHRS2.hpp"
//---------------------------------------------------------------------------------------


#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <cstdlib>
#include <iostream>

// include libraries for sensor (IMU)
#include "Navio/MPU9250.h"
#include "AHRS2.hpp"

// include lilbrary for PWM output
#include "Navio/PWM.h"
#include "Navio/Util.h"

using namespace std;

// these are called enumerations (think of them like constants), they are used to make
// code which contains calculations easier to read
#define G_SI	9.80655
#define PI	3.14159

// define servo pin locations
#define TILT	0
#define PAN	1

// define servo saturation limits
#define MAX_DUTY_CYCLE 0.6
#define CENTER 1.5





//=======================================================================================
//=======================================================================================
// BEGIN STUDENT SECTION
//=======================================================================================
//=======================================================================================

// change this to level the servo to the Navio since the alignment is not perfect
const float tilt_bias = -0.1;
const float yaw_bias  =  0.1;

// change this to modify how much the servo moves in relation to the Navio pitch angle
const float degree_to_pwm = .012;

//=======================================================================================
//=======================================================================================
// END STUDENT SECTION
//=======================================================================================
//=======================================================================================




// set delay for servo actuation until attitude estimate has settled down
float delay = 4.8; // in samples, this should be about 4 seconds
int counter = 0; // easy way to calculate delay, while loop iterates this variable

// create variable which will be used to command the servo to a certain position
float tilt_servo_cmd;

// create a pointer called "imu" to an object with type "InertialSensor"
InertialSensor *imu;
// create an AHRS object called "madgwick"
AHRS mad;

// create variables to hold the IMU output
float ax, ay, az; // 3 axis acceleromter
float gx, gy, gz; // 3 axis rate gyroscope
float mx, my, mz; // 3 axis magnetometer

// create quaternion objects (don't worry about this right now)
//float qw, qx, qy, qz;

// Euler Angles (this is what you will use for orientation information)
float roll, pitch, yaw; //body rotations about NED frame (x-y-z order)

// variables needed for the timing loop
float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;


void imu_setup()
{

	// modified from Emlid code here to only work with the mpu sensor
	cout << "Initializing IMU" << endl;
	imu = new MPU9250();
	imu -> initialize();

	cout << "Calibrating Gyro, do not move the Navio" << endl;
	for(int i = 0; i < 100; i++)
	{
		// imu object requires call to update first
		imu->update();
		// followed by call to the specific sensor read function you are working with
		imu->read_gyroscope(&gx, &gy, &gz);

		// read in gyroscope values and create a running sum
		offset[0] += -gx;
		offset[1] += -gy;
		offset[3] += -gz;

		// wait for 10,000 microseconds
		usleep(10000);
	}

	// calculate averages
	offset[0] /= 100.0;
	offset[1] /= 100.0;
	offset[2] /= 100.0;

	mad.setGyroOffset(offset[0], offset[1], offset[2]);
	cout << "Gyro calibration completed" << endl;
}

void imu_loop()
{
	// set up the timing loop
	// delta time is needed for the state estimate and this code allows dt to be
	// computed dynamically so that the dt being used in the state estimate is the
	// measured instantaneous dt instead of an average
	gettimeofday(&tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;
	if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
        gettimeofday(&tv,NULL);
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;

	imu -> update();
	imu -> read_accelerometer(&ax, &ay, &az);
	imu -> read_gyroscope(&gx, &gy, &gz);
	imu -> read_magnetometer(&mx, &my, &mz);

	// unit conversions
	// convert g to m/s
	ax /= G_SI;
	ay /= G_SI;
	az /= G_SI;
	// convert rad/s to deg/s
	gx *= 180/PI;
	gy *= 180/PI;
	gz *= 180/PI;

	// update the state estimate (quaternions)
	mad.updateMadgwick(ax, ay, az, gx/(180/PI), gy/(180/PI), gz/(180/PI), mx, my, mz,  dt);

	// get Euler angles
	mad.getEuler(&roll, &pitch, &yaw);

	// ignore the first time cycle (dt will not be correct)
	if(!isFirst)
	{
		if(dt > maxdt) maxdt = dt;
		if(dt < mindt) mindt = dt;
	}
	isFirst = 0;

	// output information to the console at a lowered rate
	dtsumm += dt;
	if(dtsumm > 0.05)
	{
		// Console output
		printf("Madgwick: ROLL: %+05.2f\t PITCH: %+05.2f\t YAW: %+05.2f\t PERIOD %.4fs\t RATE %dHz \n", roll, pitch, yaw * -1, dt, int(1/dt));
		dtsumm = 0;
	}
}


int main()
{
	imu_setup();


	cout << "Initializing Servo Output" << endl;
	PWM pwm_out;

	pwm_out.init(PAN);
	pwm_out.init(TILT);

	pwm_out.enable(PAN);
	pwm_out.enable(TILT);

	pwm_out.set_period(PAN, 50);
	pwm_out.set_period(TILT, 50);

	sleep(1);

	cout << "Servo output ready." << endl;

	while(1)
	{
		imu_loop();

		// Don't move the servo in the beginning when the state estimate is crap
		if(counter >= delay*1000)
		{
			tilt_servo_cmd = pitch * degree_to_pwm;
			tilt_servo_cmd = tilt_servo_cmd + tilt_bias + CENTER;
			counter = (delay*1000)+1;
		} else
		{
			tilt_servo_cmd = CENTER + tilt_bias;
		}


		// saturation block to prevent actuator damage
		if( tilt_servo_cmd > (CENTER+MAX_DUTY_CYCLE) )
		{
			tilt_servo_cmd = CENTER+MAX_DUTY_CYCLE;
		}
		else if( tilt_servo_cmd < (CENTER - MAX_DUTY_CYCLE) )
		{
			tilt_servo_cmd = (CENTER - MAX_DUTY_CYCLE);
		}

		// write the position command for the tilt servo
		// command is relative to 1.5 where 1.5 is the servo center position
		pwm_out.set_duty_cycle(TILT, tilt_servo_cmd);
		// pan servo always at the middle position for now
		pwm_out.set_duty_cycle(PAN, CENTER + yaw_bias);

		counter ++;
	}

	return 0;
}
