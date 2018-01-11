// custom function includes
#include "Navio/ScaleVars.h" // functions for re-scaling a value within a specified output range
// log file includes
#include <fstream>
#include <string>
// standard includes
#include <cstdlib>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
// Navio2 includes
// barometer includes
#include "Navio/MS5611.h"
#include <cmath>
// RC Input includes
#include <Navio/RCInput.h>
// IMU Includes
#include "Navio/MPU9250.h"
#include "Navio/LSM9DS1.h"
// PWM Output Includes
#include "Navio/PWM.h"
#include "Navio/Util.h"
// AHRS Includes
#include "AHRS2.hpp"
using namespace std;

const bool dbmsg_global = false; // set flag to display all debug messages
bool dbmsg_local  = false; // change in the code at a specific location to view local messages only

//------------------------------------------------------------------------------------------------------------System ID Variables
float A    = 0;
float B    = 0;
float C    = 0;
float wn   = 0;
float zeta = 1;

//-------------------------------------------------------------------------------------------------------Common Control Variables
float yaw_desired    = 0;
float yaw_error      = 0 , yaw_error_previous = 0;
float yaw_error_sum  = 0;
float yaw_error_rate = 0;

//------------------------------------------------------------------------------------------------------------------PID Variables
char control_type = 'm'; // valid options:  p = PID , n = NDI , g = glide, no spin, 'm' = multisine , 's' = step input , c = rc control
// 8:15 gains - starting point
float kp    = -.03246;
float ki    = -.03494;
float kd    = -.007003;

//---------------------------------------------------------------------------------------------------------------Step Input Timer
int step_counter;
#define NUM_STEPS 16
#define STEP_SIZE 0.025
#define INITIAL_DEFLECTION .175

//-------------------------------------------------------------------------------------------Loop timing (scheduler) Declarations

#define NUM_LOOPS  9
bool main_loop_active = true; // will bind this to an RC channel later for online tuning
struct timeval time_obj;
unsigned long long tse; // time since epoch (us)
unsigned long long time_start; // time from the beginning of current program executing (us)
unsigned long long time_now; // current time since the program starting executing (us)

//---------------------------------------------------------------------------------------------------------Barometer Declarations

int baro_step = 0; // declare the step counter (used to delay time from read to caclulate)
MS5611 barometer; // declare a new barometer object

// using the barometric equation in region 1 (valid for elevations up to 36,000ft)
// see wikipedia.org/wiki/Barometric_formula for more information
// using imperial units (some values use strange mixed units, Kelvin with imperial length?), has been tested and output looks ok
// CONSTANTS
const float Pb =  29.92126;    // static reference pressure for barometric equation (inHg)
const float Tb = 288.815;      // reference temperature for barometric equation (K) !!!not currently using this in the calculation
const float Lb =   -.0019812;  // standard temperature lapse rate (K/ft)
const float hb =   0.0;        // refernce height (ft), could be omitted but kept for generality
const float  R =   8.9494596e4;// universal gas constant (lbft^2)/(lbmolKs^2)
const float g0 =  32.17405;    // acceleration due to earth's gravity near the surface (ft/s^2)
const float  M =  28.9644;     // molar mass of earth's air (lb/lbmol)
// VARIABLES
float Tc  = 0.0; // temperature (C) <--read directly from sensor
float Tf  = 0.0; // temperature (F) <--this is not used for any calcs
float Tk  = 0.0; // temperature (K) <--converted for use in barometric equation
float Pm  = 0.0; // pressure (mbar) <--read directly from sensor
float Phg = 0.0; // pressure (inHg) <--converted for use in barometric equation
float msl = 0.0; // mean sea level altitue (ft) [should be close to 920ft for UMKC Flarsheim]

//----------------------------------------------------------------------------------------------------------RC Input Declarations
RCInput rcinput{};
const float input_range[2] = {1088,1940}; // range is the same for all channels
//const float output_range1[2] = {-.5,.5};
//const float output_range2[2] = {-.5,.5};
//const float output_range3[2] = {-100,100};
const float output_range[6][2] = {{-.25,.25},{-.25,.25},{-.25,.25},{-.25,.25},{-100,100},{0,2}};
float coefficients[6][2];

//---------------------------------------------------------------------------------------------------------------IMU Declarations
// vars to hold mpu values
float a_mpu[3] , a_mpu_ahrs[3];
float g_mpu[3] , g_mpu_ahrs[3];
float m_mpu[3] ;
// vars to hold lsm values
float a_lsm[3] , a_lsm_ahrs[3];
float g_lsm[3] , g_lsm_ahrs[3];
float m_lsm[3] ;
//create simple gyro integration for yaw as a fallback
float yaw_mpu_integrated = 0 , yaw_lsm_integrated = 0;
float yaw_mpu_integrated_degrees = 0 , yaw_lsm_integrated_degrees = 0;
float yaw_mpu_integrated_previous = 0 , yaw_lsm_integrated_previous = 0;
//also need a simple que to store old data points
float gyro_z_lsm_old[3] = {0,0,0};
float gyro_z_mpu_old[3] = {0,0,0};

//-------------------------------------------------------------------------------------------------------------Servo Declarations
#define WINCH_RIGHT 1 // right hand winch servo, 1 is the servo rail position
#define WINCH_LEFT 0 // left hand winch servo, 0 is the servo rail position
#define MAX_DEFLECTION 0.500
#define LINE_NEUTRAL 1.500
#define LINE_OFFSET -0.20
#define DEFLECTION_LIMIT .4
float winch_right_cmd = 0;
float winch_left_cmd = 0;

//--------------------------------------------------------------------------------------------------------------AHRS Declarations
#define PI 3.14159
#define G_SI 9.81
AHRS ahrs_mpu_mahony; // create a new object of type AHRS for the Mahony filter
AHRS ahrs_lsm_mahony;
AHRS ahrs_mpu_madgwick; // create a new object of type AHRS for the Madgwick filter
AHRS ahrs_lsm_madgwick;
float roll_mpu_mahony   = 0 , pitch_mpu_mahony   = 0 , yaw_mpu_mahony   = 0; // euler angles for mpu mahony
float roll_lsm_mahony   = 0 , pitch_lsm_mahony   = 0 , yaw_lsm_mahony   = 0; // euler angles for lsm mahony
float roll_mpu_madgwick = 0 , pitch_mpu_madgwick = 0 , yaw_mpu_madgwick = 0; // euler angles for mpu madgwick
float roll_lsm_madgwick = 0 , pitch_lsm_madgwick = 0 , yaw_lsm_madgwick = 0; // euler angles for lsm mahony
float offset_mpu[3];
float offset_lsm[3];
float dt, maxdt;
float mindt = 0.01;
float dtsumm = 0;
int isFirst = 1;
//Spin Tunnel Offsets
float mag_offset_mpu[3] = {44.34,24.59,58.18};
float mag_rotation_mpu[3][3] = {{.506,-.089,.022},{-.089,.798,-.040},{.229,-.040,.856}};
float mag_offset_lsm[3] = {6.43,19.72,44.47};
float mag_rotation_lsm[3][3] = {{.485,-.0065,.134},{-.0065,.812,.079},{.134,.0793,.0932}};

// Hotel Room Offsets
//float mag_offset[3] = {-14.47,13.35,85.62};
//float mag_rotation[3][3] = {{.061,-1.23,-.64},{-1.23,-1.80,1.227},{-.64,1.22,-3.13}};


//-----------------------------------------------------------------------------------------------------------Logfile Declarations
// these are used to format the filename string
#define FILENAME_LENGTH 12
#define FILENAME_OFFSET 4
// leave extra room for '.csv' and potential '-1', '-2', etc.
char filename[FILENAME_LENGTH+6];
// log file location (relative path)
string file_location = "LogFiles/";
// function to check if file exists so that the filename can be dynamically changed
bool file_exists(string name_of_file){
	ifstream checkfile(name_of_file); // try to read from the file
	return checkfile;} // cast the result to bool, if file opens, this returns true (it exists)

int main( int argc , char *argv[])
{
	//----------------------------------------------------------------------------------------------------------------File Title Prefix
	kp = kp*.0175; // convert to radians
	ki = ki*.0175;
	kd = kd*.0175;
	int multisine_counter = 0;
	int parameter;
	char *logfile_prefix;
	string logfile_prefix_string;
	logfile_prefix_string = file_location;
	while((parameter = getopt(argc,argv, "hd:")) != -1)
	{
		switch(parameter)
		{
			case 'h' : cout << "Use option \"-d <FilePrefix>\" to insert a prefix into the filename" << endl; return EXIT_FAILURE;
			case 'd' : logfile_prefix = optarg; logfile_prefix_string = file_location+string(logfile_prefix)+'_'; break;
			case '?' : cout << "Invalid option.  Use -h for help" << endl; return EXIT_FAILURE;
		}
	}



	//-------------------------------------------------------------------------------------------Loop timing (scheduler) Initialization
	// initialize the time variables
	gettimeofday(&time_obj, NULL);
	tse          = time_obj.tv_sec*1000000LL + time_obj.tv_usec; // time since epoch (us)
	//tse        = time_obj.tv_sec*1000LL + time_obj.tv_usec/1000; // time since epoch (ms)
	time_start = tse; // time since epoch (ms), will be used to calculate time since start (beginning at 0us)

	// declare the scheduler frequencies
	cout << endl;
	cout << "Establishing loop frequencies.........." << endl;
	const float frequency [NUM_LOOPS] = {300,1,1,100,50,20,10,.05,1}; //Hz
	long long duration [NUM_LOOPS]; // will store the expected time since last execution for a given loop
	int timer [NUM_LOOPS]; // will store the time since the last execution in a given loop
	int watcher [NUM_LOOPS]; // will be used for debugging

	// calculate the scheduler us delay durations and populate the timer array with zeros
	cout << "Calculating loop delay durations......." << endl;
	cout << "Populating loop timers................." << endl;
	for(int i = 0 ; i < NUM_LOOPS ; i++){
		duration[i] = 1000000/frequency[i];
		//duration[i] = 1000/frequency[i];
		if(dbmsg_global || dbmsg_local){cout << "Frequency is " << frequency[i] << "(Hz), duration is " << duration[i] << "(us)"<< endl;}
		timer[i] = 0;
		if(dbmsg_global || dbmsg_local){cout << "Timer is set to " << timer[i] << "(us), should all be zero" << endl;}
		watcher[i] = 0;}
	//---------------------------------------------------------------------------------------------------------Barometer Initialization
	cout << "Initializing barometer................." << endl;
	barometer.initialize();
	cout << " --Barometer successfully initalized-- " << endl;
	//----------------------------------------------------------------------------------------------------------RC Input Initialization
	cout << "Initilizing RC Input..................." << endl;
	rcinput.init();
	int rc_array [rcinput.channel_count]; // array for holding the values which are read from the controller
	float rc_array_scaled [rcinput.channel_count]; // array for holding the scaled values
	for(int i = 0 ; i < rcinput.channel_count ; i++ ){
		rc_array[i] = rcinput.read(i);} // read in each value using the private class function (converts to int automatically)
	cout << "Currently using " << rcinput.channel_count << " channels............" << endl;
	cout << " --RC Input successfully initialized-- " << endl;
	cout << "Creating RC Input range coefficients   " << endl;
	for(int i = 0 ; i < rcinput.channel_count ; i++){
		coefficients[i][0] = calculate_slope(input_range,output_range[i]);
		//cout << coefficients[i][0];
		coefficients[i][1] = calculate_intercept(input_range,output_range[i],coefficients[i][0]);
		//cout << coefficients[i][1] << endl;
	}
	cout << "Succesffuly created range coefficients " << endl;
	//---------------------------------------------------------------------------------------------------------------IMU Initialization
	cout << "Initializing IMUs......................" << endl;
	InertialSensor *mpu;
	mpu = new MPU9250();
	mpu->initialize();
	cout << " --MPU9250 successfully initialized!--" << endl;
	InertialSensor *lsm;
	lsm = new LSM9DS1();
	lsm->initialize();
	cout << " --LSM9DS1 successfully initialized!--" << endl;
	cout << "Populating IMU sensor arrays.........." << endl;
	for(int i = 0; i < 3 ; i++){
		a_mpu[i]=g_mpu[i]=m_mpu[i]=a_lsm[i]=g_lsm[i]=m_lsm[i] = 0.0;}
	//-------------------------------------------------------------------------------------------------------------Servo Initialization
	cout << "Initializing PWM Output................" << endl;
	PWM pwm_out;
	if(!pwm_out.init(WINCH_RIGHT)){ // right hand side winch servo
		cout << "Cannot Initialize East Side Winch Servo" << endl;
		cout << "Make sure you are root" << endl;
		return 0;}
	if(!pwm_out.init(WINCH_LEFT)){ // left hand side winch servo
		cout << "Cannot Initialize West Side Winch Servo" << endl;
		cout << "Make sure you are root" << endl;
		return 0;}
	cout << "Enabling PWM output channels..........." << endl;
	pwm_out.enable(WINCH_RIGHT); // both init() and enable() must be called to use the PWM output on a particular pin
	pwm_out.enable(WINCH_LEFT);
	cout << "Setting PWM period for 50Hz............" << endl;
	pwm_out.set_period(WINCH_RIGHT , 50); // set the PWM frequency to 50Hz
	pwm_out.set_period(WINCH_LEFT , 50);
	cout << "  --PWM Output successfully enabled-- " << endl;
	//-------------------------------------------------------------------------------------------------------------AHRS Initialization
	cout << "Initializing gyroscope................." << endl;
	cout << "Reading gyroscope offsets.............." << endl;
	for(int i = 0 ; i < 100 ; i++)
	{
		mpu->read_gyroscope(&g_mpu[0],&g_mpu[1],&g_mpu[2]);
		lsm->read_gyroscope(&g_lsm[0],&g_lsm[1],&g_lsm[2]);
		//read 100 samples from each gyroscope axis on each sensor
		g_mpu[0] *= 180/PI;
		g_mpu[1] *= 180/PI;
		g_mpu[2] *= 180/PI;
		g_lsm[0] *= 180/PI;
		g_lsm[1] *= 180/PI;
		g_lsm[2] *= 180/PI;
		//populate the offset arrays
		offset_mpu[0] += (-g_mpu[0]*0.0175);
		offset_mpu[1] += (-g_mpu[1]*0.0175);
		offset_mpu[2] += (-g_mpu[2]*0.0175);
		offset_lsm[0] += (-g_lsm[0]*0.0175);
		offset_lsm[1] += (-g_lsm[1]*0.0175);
		offset_lsm[2] += (-g_lsm[2]*0.0175);
		//wait a bit before reading gyro again
		usleep(10000);
	}
	cout << "Calculating gyroscope offsets.........." << endl;
	// average the offsets for the mpu
	offset_mpu[0]/=100.0;
	offset_mpu[1]/=100.0;
	offset_mpu[2]/=100.0;
	// average the offsets for the lsm
	offset_lsm[0]/=100.0;
	offset_lsm[1]/=100.0;
	offset_lsm[2]/=100.0;
	cout << "Setting gyroscope offsets.............." << endl;
	ahrs_mpu_mahony.setGyroOffset(offset_mpu[0],offset_mpu[1],offset_mpu[2]);
	ahrs_lsm_mahony.setGyroOffset(offset_lsm[0],offset_lsm[1],offset_lsm[2]);
	ahrs_mpu_madgwick.setGyroOffset(offset_mpu[0],offset_mpu[1],offset_mpu[2]);
	ahrs_lsm_madgwick.setGyroOffset(offset_lsm[0],offset_lsm[1],offset_lsm[2]);
	cout << " --Gyroscope offsets stored in AHRS--  " << endl;
	ahrs_mpu_mahony.setMagCalibration(mag_offset_mpu,mag_rotation_mpu);
	ahrs_lsm_mahony.setMagCalibration(mag_offset_lsm,mag_rotation_lsm);
	ahrs_mpu_madgwick.setMagCalibration(mag_offset_mpu,mag_rotation_mpu);
	ahrs_lsm_madgwick.setMagCalibration(mag_offset_lsm,mag_rotation_lsm);

	//------------------------------------------------------------------------------------------------------------------Welcome Message
	usleep(500000);
	cout << endl;
	cout << "======================================" << endl;
	cout << "Initialization completed.............." << endl;
	cout << "======================================" << endl << endl;
	usleep(500000);
	cout << "======================================" << endl;
	cout << "Main Loop starting now................" << endl;
	cout << "======================================" << endl << endl;
	usleep(500000);

	//step_counter = 0;

	//----------------------------------------------------------------------------------------------------------Log File Initialization
	while(true)
	{
		int standby_message_timer = 0;
		while(rc_array[5] <= 1500){
			if(standby_message_timer > 250){
				cout << endl << "--------------------------------------" << endl << "          Autopilot Inactive         " << endl;
				cout << " Dynamic Lines at Neutral Deflection " << endl;
				cout << "        Waiting for Killswitch       " << endl << "-------------------------------------" << endl;
				standby_message_timer = 0;}
			// when the autopilot is inactive, set both winches to the neutral deflection
			pwm_out.set_duty_cycle(WINCH_RIGHT, LINE_NEUTRAL);
			pwm_out.set_duty_cycle(WINCH_LEFT, LINE_NEUTRAL);
			rc_array[5] = rcinput.read(5);
			step_counter = -1;
			usleep(5000);
			standby_message_timer++;
			//everything that needs to be set to zero by the killswitch goes here
			multisine_counter = 0;
			yaw_mpu_integrated = 0;
			yaw_mpu_integrated_previous = 0;
			yaw_lsm_integrated = 0;
			yaw_lsm_integrated_previous = 0;
			for(int i = 0 ; i < 3 ; i++){
				gyro_z_lsm_old[i] = 0;
				gyro_z_mpu_old[i] = 0;
			}
			yaw_error_sum       = 0; //prevent integral wind up
			yaw_error           = 0;
			yaw_error_previous  = 0;
		}
		time_t result = time(NULL);
		char *today = asctime(localtime(&result)); // establish char array to write today's date
		today[strlen(today) - 1] = '\0'; // remove end line from the end of the character array
		// remove spaces, remove colons, only keep the month/day/hour/minute
		for(int i = 0 ; i < FILENAME_LENGTH ; i++){
			if(today[i+FILENAME_OFFSET] == ' ' || today[i+FILENAME_OFFSET] == ':'){
				if(i == 8 - FILENAME_OFFSET) {
					filename[i] = '0';} // add leading zero in front of 1 digit day
				else{
					filename[i] = '_';}}
			else{
				filename[i] = today[i+FILENAME_OFFSET];}}
		// add .csv to the end of the file
		filename[FILENAME_LENGTH+0] = '-';
		filename[FILENAME_LENGTH+1] = '0';
		filename[FILENAME_LENGTH+2] = '.';
		filename[FILENAME_LENGTH+3] = 'c';
		filename[FILENAME_LENGTH+4] = 's';
		filename[FILENAME_LENGTH+5] = 'v';
		// for adding the relative path to the file, it will be converted to a string
		string filename_str(filename);
		//cout << "filename_str" << filename_str << endl;
		filename_str = logfile_prefix_string+filename_str;
		// used to put a number on the end of the file if it is a duplicate
		int filename_index = 1;
		// loop while the file name already exists or until we are out of indices to write to
		cout << endl << "Setting up log file...................." << endl;
		cout << "Checking for unique file name.........." << endl;
		while(file_exists(filename_str))
		{
		cout << "Filename already exists................" << endl;
			filename[FILENAME_LENGTH+0] = '-';
			filename[FILENAME_LENGTH+1] = filename_index + '0';
			filename[FILENAME_LENGTH+2] = '.';
			filename[FILENAME_LENGTH+3] = 'c';
			filename[FILENAME_LENGTH+4] = 's';
			filename[FILENAME_LENGTH+5] = 'v';
			string temp(filename);
			filename_str = logfile_prefix_string+temp;
			filename_index++;
			usleep(50000);} // don't try to read the files at warp speed
		// create the file output stream object
		ofstream fout;
		// open the file
		cout << "Log file created at: " << endl << filename_str << endl << endl;
		fout.open(filename_str,ios::out);
		// header string
		fout << "today,microseconds_since_start,"
			"msl,rc0,rc1,rc2,rc3,rc4,rc5,"
			"rc0_scaled,rc1_scaled,rc2_scaled,rc3_scaled,rc4_scaled,rc5_scaled,"
			"roll_mpu_mahony,pitch_mpu_mahony,yaw_mpu_mahony,"
			"roll_lsm_mahony,pitch_lsm_mahony,yaw_lsm_mahony,"
			"roll_mpu_madgwick,pitch_mpu_madgwick,yaw_mpu_madgwick,"
			"roll_lsm_madgwick,pitch_lsm_madgwick,yaw_lsm_madgwick,"
			"a_mpu[0],a_mpu[1],a_mpu[2],a_lsm[0],a_lsm[1],a_lsm[2],"
			"g_mpu[0],g_mpu[1],g_mpu[2],g_lsm[0],g_lsm[1],g_lsm[2],"
			"m_mpu[0],m_mpu[1],m_mpu[2],m_lsm[0],m_lsm[1],m_lsm[2],"
			"winch_right_cmd,winch_left_cmd,A,B,C,wn,zeta,kp,ki,kd"
			"yaw_desired,yaw_error,yaw_error_previous,yaw_error_rate,yaw_error_sum" << endl;
		usleep(2000000);

	while(rc_array[5]>1500)
	{
		// refresh time now to prepare for another loop execution
		gettimeofday(&time_obj, NULL); // must first update the time_obj
		//tse        = time_obj.tv_sec*1000LL + time_obj.tv_usec/1000; // update tse (ms)
		tse        = time_obj.tv_sec*1000000LL + time_obj.tv_usec; // update tse (us)
		time_now   = tse - time_start; // calculate the time since execution start by subtracting off tse

		if( (time_now-timer[0]) > duration[0])
		{

			//----------------------------------------------------------------------------------------------------------------AHRS Update
			dt = time_now-timer[0];
			dt = dt/1000000.0; // convert from useconds
			// Tested sampling rate for IMUs, with both IMUs execution of the following block was taking
			// approximate 1300us (~750Hz), slowed this loop down ot 500Hz so that there is a little
			// headroom but the sensor is being polled as quickly as possible
			// Read both IMUs at 500Hz
			// start with the MPU9250
			mpu->update();
			mpu->read_accelerometer(&a_mpu[0],&a_mpu[1],&a_mpu[2]);
			mpu->read_gyroscope(&g_mpu[0],&g_mpu[1],&g_mpu[2]);
			mpu->read_magnetometer(&m_mpu[0],&m_mpu[1],&m_mpu[2]);

			// now read in the LSM9DS1
			lsm->update();
			lsm->read_accelerometer(&a_lsm[0],&a_lsm[1],&a_lsm[2]);
			lsm->read_gyroscope(&g_lsm[0],&g_lsm[1],&g_lsm[2]);
			lsm->read_magnetometer(&m_lsm[0],&m_lsm[1],&m_lsm[2]);

			for(int i = 0 ; i < 3 ; i++ ){
				a_mpu_ahrs[i] = a_mpu[i]/G_SI;
				g_mpu_ahrs[i] = g_mpu[i]*(180/PI)*0.0175;
				a_lsm_ahrs[i] = a_lsm[i]/G_SI;
				g_lsm_ahrs[i] = g_lsm[i]*(180/PI)*0.0175;}

			ahrs_mpu_mahony.updateMahony(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0],g_mpu_ahrs[1],g_mpu_ahrs[2],m_mpu[0],m_mpu[1],m_mpu[2],dt);
			ahrs_lsm_mahony.updateMahony(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0],g_lsm_ahrs[1],g_lsm_ahrs[2],m_lsm[0],m_lsm[1],m_lsm[2],dt);
			ahrs_mpu_madgwick.updateMadgwick(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0],g_mpu_ahrs[1],g_mpu_ahrs[2],m_mpu[0],m_mpu[1],m_mpu[2],dt);
			ahrs_lsm_madgwick.updateMadgwick(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0],g_lsm_ahrs[1],g_lsm_ahrs[2],m_lsm[0],m_lsm[1],m_lsm[2],dt);
//			ahrs_mpu_mahony.updateMahonyIMU(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0],g_mpu_ahrs[1],g_mpu_ahrs[2],dt);
//			ahrs_lsm_mahony.updateMahonyIMU(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0],g_lsm_ahrs[1],g_lsm_ahrs[2],dt);
//			ahrs_mpu_madgwick.updateMadgwickIMU(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0],g_mpu_ahrs[1],g_mpu_ahrs[2],dt);
//			ahrs_lsm_madgwick.updateMadgwickIMU(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0],g_lsm_ahrs[1],g_lsm_ahrs[2],dt);

			//propogate the que holding the old gyro values
			gyro_z_mpu_old[2] = gyro_z_mpu_old[1];
			gyro_z_mpu_old[1] = gyro_z_mpu_old[0];
			gyro_z_mpu_old[0] = g_mpu_ahrs[2]-offset_mpu[2];
			gyro_z_lsm_old[2] = gyro_z_lsm_old[1];
			gyro_z_lsm_old[1] = gyro_z_lsm_old[0];
			gyro_z_lsm_old[0] = g_lsm_ahrs[2]-offset_lsm[2];

			yaw_mpu_integrated_previous = yaw_mpu_integrated;
			yaw_lsm_integrated_previous = yaw_lsm_integrated;

			//update the integrated gyro yaw (this is a fallback in case the heading is bad)
			yaw_mpu_integrated = (((gyro_z_mpu_old[0]+gyro_z_mpu_old[1])/2)*dt) + yaw_mpu_integrated_previous;
			yaw_mpu_integrated_degrees = yaw_mpu_integrated * (180/PI);
			yaw_lsm_integrated = (((gyro_z_lsm_old[0]+gyro_z_lsm_old[1])/2)*dt) + yaw_lsm_integrated_previous;
			yaw_lsm_integrated_degrees = yaw_lsm_integrated * (180/PI);

			// Debug message, these slow down execution considerably
			// with messages enabled this loop may not operate at the expected frequency
			if(dbmsg_global || dbmsg_local) {cout << "This is the " << frequency[0] << "(Hz) loop." << endl
				<< "Expected time since last execution " << duration[0]
				<< "(us), Actual time since last execution " << time_now-timer[0] << "(us)" << endl;}
			watcher[0] = time_now - timer[0]; // used to check loop frequency
			timer[0] = time_now;
		}

		if( (time_now-timer[1]) > duration[1])
		{

			// Debug message, these slow down execution considerably
			// with messages enabled this loop may not operate at the expected frequency
			if(dbmsg_global || dbmsg_local) {cout << "This is the " << frequency[1] << "(Hz) loop." << endl
				<< "Expected time since last execution " << duration[1]
				<< "(us), Actual time since last execution " << time_now-timer[1] << "(us)" << endl;}
			watcher[1] = time_now - timer[1]; // used to check loop frequency
			timer[1] = time_now;
		}

		if( (time_now-timer[2]) > duration[2])
		{


			// Debug message, these slow down execution considerably
			// with messages enabled this loop may not operate at the expected frequency
			if(dbmsg_global || dbmsg_local) {cout << "This is the " << frequency[2] << "(Hz) loop." << endl
				<< "Expected time since last execution " << duration[2]
				<< "(us), Actual time since last execution " << time_now-timer[2] << "(us)" << endl;}
			watcher[2] = time_now - timer[2]; // used to check loop frequency
			timer[2] = time_now;
		}

		if( (time_now-timer[3]) > duration[3])
		{

			//-------------------------------------------------------------------------------------------------------------Barometer Read
			if(baro_step == 0){
				barometer.refreshPressure();}
			else if(baro_step == 1){
				barometer.readPressure();}
			else if(baro_step == 2){
				barometer.refreshTemperature();}
			else if(baro_step == 3){
				barometer.readTemperature();}
			else if(baro_step == 4){
				barometer.calculatePressureAndTemperature();
				//cout << "baro update complete, current time: " << time_now << "(us)" << endl; // full update every 0.04s
				baro_step = -1;}
			else{
				cout << "improper barometer step, pressure may be incorrect" << endl;}
			baro_step++; // increment the step

			//--------------------------------------------------------------------------------------------------------------RC Input Read
			for(int i = 0 ; i < rcinput.channel_count ; i++ )
			{
				rc_array[i] = rcinput.read(i); // read in each value using the private class function (converts to int automatically)
				if(dbmsg_global || dbmsg_local) {cout << "Channel Number: " << i << " Channel Value: "  << rc_array[i] << endl;}
			}
			for(int i = 0 ; i < rcinput.channel_count ; i++)
			{
				rc_array_scaled[i] = scale_output(coefficients[0],float(rc_array[i]));
			}

			//-----------------------------------------------------------------------------------------------------------------PWM Output
			// these winch positions represent minimum line lengths (maximum deflection)
			//winch_right_cmd = LINE_NEUTRAL+MAX_DEFLECTION;
			//winch_left_cmd = LINE_NEUTRAL+MAX_DEFLECTION;

			// these represent maximum line release (minimum line pull, minimum deflection)
			//winch_right_cmd = LINE_NEUTRAL - MAX_DEFLECTION;
			//winch_left_cmd = LINE_NEUTRAL - MAX_DEFLECTION;

			// PWM neutral position
			//winch_right_cmd = LINE_NEUTRAL;
			//winch_left_cmd = LINE_NEUTRAL;

			// RC control, both servos on roll stick
			//winch_right_cmd = LINE_NEUTRAL + rc_array_scaled[0];
			//winch_left_cmd  = LINE_NEUTRAL + rc_array_scaled[0];





			//------------------------------------------------------------------------------------------------AHRS Euler Angle Conversion
			ahrs_mpu_mahony.getEuler(&roll_mpu_mahony,&pitch_mpu_mahony,&yaw_mpu_mahony);
			ahrs_lsm_mahony.getEuler(&roll_lsm_mahony,&pitch_lsm_mahony,&yaw_lsm_mahony);
			ahrs_mpu_madgwick.getEuler(&roll_mpu_madgwick,&pitch_mpu_madgwick,&yaw_mpu_madgwick);
			ahrs_lsm_madgwick.getEuler(&roll_lsm_madgwick,&pitch_lsm_madgwick,&yaw_lsm_madgwick);

			//----------------------------------------------------------------------------------------------------------------Controllers
			float dt_control = time_now-timer[3]; // dt for this loop
			dt_control = dt_control/1000000.0; // convert from useconds

			yaw_desired           = 0; // set yaw desired here
			yaw_error_previous    = yaw_error;
			yaw_error             = yaw_desired - yaw_lsm_integrated_degrees;
			yaw_error_rate        = (yaw_error - yaw_error_previous)/dt_control;
			yaw_error_sum         = (((yaw_error + yaw_error_previous)*dt_control)/2) + yaw_error_sum;

			// hack to avoid changing the variable name "t" in the multisine
			float t = time_now*1e-6;

			switch(control_type){
				case 'p':
					//cout << "PID active" << endl;
					winch_left_cmd = LINE_NEUTRAL + kp*yaw_error + ki*yaw_error_sum + kd*yaw_error_rate;
					winch_right_cmd = LINE_NEUTRAL - LINE_OFFSET;
					break;
				case 'n':
					//cout << "NDI active" << endl;
					break;
				case 'r': // minimum deflection, for rigging
					//cout << "rigging active" << endl;
					winch_right_cmd = LINE_NEUTRAL + MAX_DEFLECTION;
					winch_left_cmd = LINE_NEUTRAL - MAX_DEFLECTION;
					break;
				case 'g': // glide, both lines pulled
					//cout << "neutral glide" << endl;
					winch_right_cmd = LINE_NEUTRAL - LINE_OFFSET;
					winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET;
					break;
				case 's': // step input
					if(rc_array[4] > 1500)
					{
						if(step_counter >= 0){
							winch_right_cmd = LINE_NEUTRAL+INITIAL_DEFLECTION - step_counter*STEP_SIZE;
							if((step_counter+1)%2 < 1){
								//cout << "odd number" << endl;
								winch_right_cmd = LINE_NEUTRAL;
							}}
						else{
							winch_right_cmd = LINE_NEUTRAL;}
						winch_left_cmd = LINE_NEUTRAL+LINE_OFFSET;
					} else
					{
						if(step_counter>=0){
							winch_left_cmd = LINE_NEUTRAL-INITIAL_DEFLECTION + step_counter*STEP_SIZE;
							if((step_counter+1)%2 < 1){
								//cout << "odd number" << endl;
								winch_left_cmd = LINE_NEUTRAL;
							}}
						else{
							winch_left_cmd = LINE_NEUTRAL;}
						winch_right_cmd = LINE_NEUTRAL-LINE_OFFSET;
					}
					break;
				case 'c': // rc control
					//for RC control, throttle is left servo, elevator is right servo, down on the sticks is line pull
					winch_right_cmd = LINE_NEUTRAL - rc_array_scaled[1];
					winch_left_cmd = LINE_NEUTRAL - rc_array_scaled[2];
					break;
				case 'm': // multisine
					//cout << "multisine active" << endl;

					// multisine or step input, only enable when switch D is in the 1 or 2 position
					if(rc_array[4] > 1500) // switch A 0 for left, 1 for right
					{
						if(multisine_counter < 61){
							// right winch is dynamic, left winch is static
							winch_right_cmd = LINE_NEUTRAL-LINE_OFFSET-.04*(sin(.2094*t-0.6111)+sin(.4189*t-1.3593)+sin(0.8976*t-.8035)+sin(2.0944*t+2.6684)+sin(2.8274*t-1.2069)+sin(3.7699*t+2.8186)+sin(4.7124*t-.8956)+sin(5.3407*t+1.8869)+sin(6.2832*t+2.1591));
							winch_left_cmd = LINE_NEUTRAL+LINE_OFFSET;
						}
						else{
							// after 60 seconds disable both and return to servo neutral
							winch_right_cmd = LINE_NEUTRAL;
							winch_left_cmd = LINE_NEUTRAL;
						}
						// uncomment for step input
						//if(step_counter >= 0){
						//	winch_right_cmd = LINE_NEUTRAL+INITIAL_DEFLECTION - step_counter*STEP_SIZE;
						//	if((step_counter+1)%2 < 1){
						//		//cout << "odd number" << endl;
						//		winch_right_cmd = LINE_NEUTRAL;
						//	}}
						//else{
						//	winch_right_cmd = LINE_NEUTRAL;}
						//winch_left_cmd = LINE_NEUTRAL-LINE_OFFSET;
					} else	{
						// apply the same things to the left hand side if the switch is down (0 position)
						if(multisine_counter < 61){
							winch_left_cmd = LINE_NEUTRAL+LINE_OFFSET+.04*(sin(.2094*t-0.6111)+sin(.4189*t-1.3593)+sin(0.8976*t-.8035)+sin(2.0944*t+2.6684)+sin(2.8274*t-1.2069)+sin(3.7699*t+2.8186)+sin(4.7124*t-.8956)+sin(5.3407*t+1.8869)+sin(6.2832*t+2.1591));
							winch_right_cmd = LINE_NEUTRAL-LINE_OFFSET;
						}
						else{
							winch_right_cmd = LINE_NEUTRAL;
							winch_left_cmd = LINE_NEUTRAL;
						}
						// uncomment for step input
						//if(step_counter>=0){
						//	winch_left_cmd = LINE_NEUTRAL-INITIAL_DEFLECTION + step_counter*STEP_SIZE;
						//	if((step_counter+1)%2 < 1){
						//		//cout << "odd number" << endl;
						//		winch_left_cmd = LINE_NEUTRAL;
						//	}}
						//else{
						//	winch_left_cmd = LINE_NEUTRAL;}
						//winch_right_cmd = LINE_NEUTRAL+LINE_OFFSET;
				}
					break;
				default:
					// default is PWM neutral
					//cout << "Default active" << endl;
					winch_right_cmd = LINE_NEUTRAL;
					winch_left_cmd  = LINE_NEUTRAL;
					break;}

			// These are the correct directions in which to apply the offsets for line pull (glide)
			//cout << "Default active" << endl;
			//winch_right_cmd = LINE_NEUTRAL-LINE_OFFSET;
			//winch_left_cmd  = LINE_NEUTRAL+LINE_OFFSET;


			// actuator saturation, right side
			if(winch_right_cmd > LINE_NEUTRAL + DEFLECTION_LIMIT){
				winch_right_cmd = LINE_NEUTRAL + DEFLECTION_LIMIT;}
			else if(winch_right_cmd < LINE_NEUTRAL - DEFLECTION_LIMIT){
				winch_right_cmd = LINE_NEUTRAL - DEFLECTION_LIMIT;}
			// actuator saturation, left side
			if(winch_left_cmd > LINE_NEUTRAL + DEFLECTION_LIMIT){
				winch_left_cmd = LINE_NEUTRAL + DEFLECTION_LIMIT;}
			else if(winch_left_cmd < LINE_NEUTRAL - DEFLECTION_LIMIT){
				winch_left_cmd = LINE_NEUTRAL - DEFLECTION_LIMIT;}


			// always right the duty cycle, change control type by writing to the winch_cmd variable
			pwm_out.set_duty_cycle(WINCH_RIGHT, winch_right_cmd);
			pwm_out.set_duty_cycle(WINCH_LEFT, winch_left_cmd);

			//-------------------------------------------------------------------------------------------------------Data Log File Output
			fout << today << "," << time_now << "," << msl << ",";
			for(int i = 0 ; i < rcinput.channel_count ; i++){
				fout << rc_array[i] << ",";}
			for(int i = 0 ; i < rcinput.channel_count ; i++){
				fout << rc_array_scaled[i] << ",";}
			fout << roll_mpu_mahony << "," << pitch_mpu_mahony << "," << yaw_mpu_mahony << ",";
			fout << roll_lsm_mahony << "," << pitch_lsm_mahony << "," << yaw_lsm_mahony << ",";
			fout << roll_mpu_madgwick << "," << pitch_mpu_madgwick << "," << yaw_mpu_madgwick << ",";
			fout << roll_lsm_madgwick << "," << pitch_lsm_madgwick << "," << yaw_lsm_madgwick << ",";
			fout << a_mpu[0] << "," << a_mpu[1] << "," << a_mpu[2] << "," << a_lsm[0] << "," << a_lsm[1] << ",";
			fout << a_lsm[2] << "," << g_mpu[0] << "," << g_mpu[1] << "," << g_mpu[2] << "," << g_lsm[0] << "," << g_lsm[1] << ",";
			fout << g_lsm[2] << "," << m_mpu[0] << "," << m_mpu[1] << "," << m_mpu[2] << "," << m_lsm[0] << "," << m_lsm[1] << "," << m_lsm[2] << ",";
			fout << winch_right_cmd << "," << winch_left_cmd << "," << A << "," << B << "," << C << ",";
			fout << wn << "," << zeta << "," << kp << "," << ki << "," << kd;
			fout << yaw_desired << yaw_error << yaw_error_previous << yaw_error_rate << yaw_error_sum << endl;



			// Debug message, these slow down execution considerably
			// with messages enabled this loop may not operate at the expected frequency
			if(dbmsg_global || dbmsg_local) {cout << "This is the " << frequency[3] << "(Hz) loop." << endl
				<< "Expected time since last execution " << duration[3]
				<< "(us), Actual time since last execution " << time_now-timer[3] << "(us)" << endl;}
			watcher[3] = time_now - timer[3]; // used to check loop frequency
			timer[3] = time_now;
		}

		if( (time_now-timer[4]) > duration[4])
		{
			//-------------------------------------------------------------------------------------------------------------Barometer Calc
			// barometer does full update at 25Hz, no need to do the full altitude calculation any faster than 50Hz
			Tc  = barometer.getTemperature(); // temperature from sensor (C), value is recorded at surface of PCB, (higher than ambient!)
			Tk  = Tc + 273.15; // convert to Kelvin (needed for barometric formula
			Tf  = Tc * (9.0/5.0) + 32; // convert to Fahrenheit (uncomment for sanity check but it is not used for any calcs)
			Pm  = barometer.getPressure(); // pressure is natively given in mbar
			Phg = Pm*.02953; // convert to inHg
			// barometric equation
			msl = hb + (Tb/Lb)*(pow((Phg/Pb),((-R*Lb)/(g0*M)))-1); // NOTE: using local Tk instead of standard temperature (Tb)

			if( dbmsg_global || dbmsg_local) {cout << "Barometer Check (at 50Hz), Temp (C,K,F): " << Tc << " " << Tk << " " << Tf << " "
				<< "Pressure (mbar, inHg): " << Pm << " " << Phg << " Altitude MSL (ft): " << msl << endl;}

			// Debug message, these slow down execution considerably
			// with messages enabled this loop may not operate at the expected frequency
			if(dbmsg_global || dbmsg_local) {cout << "This is the " << frequency[4] << "(Hz) loop." << endl
				<< "Expected time since last execution " << duration[4]
				<< "(us), Actual time since last execution " << time_now-timer[4] << "(us)" << endl;}
			watcher[4] = time_now - timer[4]; // used to check loop frequency
			timer[4] = time_now;
		}


		if( (time_now-timer[5]) > duration[5])
		{


			// Debug message, these slow down execution considerably
			// with messages enabled this loop may not operate at the expected frequency
			if(dbmsg_global || dbmsg_local) {cout << "This is the " << frequency[5] << "(Hz) loop." << endl
				<< "Expected time since last execution " << duration[5]
				<< "(us), Actual time since last execution " << time_now-timer[5] << "(us)" << endl;}
			watcher[5] = time_now - timer[5]; // used to check loop frequency
			timer[5] = time_now;
		}

		if( (time_now-timer[6]) > duration[6])
		{


			// Debug message, these slow down execution considerably
			// with messages enabled this loop may not operate at the expected frequency
			if(dbmsg_global || dbmsg_local) {cout << "This is the " << frequency[6] << "(Hz) loop." << endl
				<< "Expected time since last execution " << duration[6]
				<< "(us), Actual time since last execution " << time_now-timer[6] << "(us)" << endl;}
			watcher[6] = time_now - timer[6]; // used to check loop frequency
			timer[6] = time_now;
		}

		if( (time_now-timer[7]) > duration[7])
		{

//			cout << "this is happening at .05 Hz, this is the step: " << step_counter << endl;

			step_counter++;
			if(step_counter>NUM_STEPS-1){
				step_counter = 0;
			}


			// Debug message, these slow down execution considerably
			// with messages enabled this loop may not operate at the expected frequency
			if(dbmsg_global || dbmsg_local) {cout << "This is the " << frequency[7] << "(Hz) loop." << endl
				<< "Expected time since last execution " << duration[7]
				<< "(us), Actual time since last execution " << time_now-timer[7] << "(us)" << endl;}
			watcher[7] = time_now - timer[7]; // used to check loop frequency
			timer[7] = time_now;
		}

		if( (time_now-timer[8]) > duration[8])
		{

//			fout << today << "," << time_now << "," << msl << ",";
//			for(int i = 0 ; i < rcinput.channel_count ; i++){
//				fout << rc_array[i] << ",";}
//			fout << a_mpu[0] << "," << a_mpu[1] << "," << a_mpu[2] << "," << a_lsm[0] << "," << a_lsm[1] << ",";
//			fout << a_lsm[2] << "," << g_mpu[0] << "," << g_mpu[1] << "," << g_mpu[2] << "," << g_lsm[0] << "," << g_lsm[1] << ",";
//			fout << g_lsm[2] << "," << m_mpu[0] << "," << m_mpu[1] << "," << m_mpu[2] << "," << m_lsm[0] << "," << m_lsm[1] << "," << m_lsm[2] << endl;





			//Sensor Output Message, use to make sure sensors outputs look reasonable
			//if(dbmsg_global || dbmsg_local)
			if(!dbmsg_global && !dbmsg_local)
			{
				cout << endl;
				//cout << "Sensor Output Message" << endl;
				cout << "Barometer Altitude: " << msl << " ft" << endl;
				cout << "Raw RC Input:";
				for(int i = 0 ; i < rcinput.channel_count ; i++ )
				{
					cout << " channel " << i << ": " << rc_array[i];
					if(i != rcinput.channel_count -1){
						cout << ",";}
				}
				cout << endl;
				cout << "Scaled RC Input:";
				for(int i = 0 ; i < rcinput.channel_count ; i++ )
				{
					cout << " channel " << i << ": " << rc_array_scaled[i];
					if(i != rcinput.channel_count -1){
						cout << ",";}
				}
				cout << endl;
				cout << "MPU9250: ";
				cout << "Accelerometer: " << a_mpu[0] << " " << a_mpu[1] << " " << a_mpu[2];
				cout << " Gyroscope: " << g_mpu[0] << " " << g_mpu[1] << " " << g_mpu[2];
				cout << " Magnetometer: " << m_mpu[0] << " " << m_mpu[1] << " " << m_mpu[2] << endl;

				cout << "LSM9DS1: ";
				cout << "Accelerometer: " << a_lsm[0] << " " << a_lsm[1] << " " << a_lsm[2];
				cout << " Gyroscope: " << g_lsm[0] << " " << g_lsm[1] << " " << g_lsm[2];
				cout << " Magnetometer: " << m_lsm[0] << " " << m_lsm[1] << " " << m_lsm[2] << endl;

				cout << "PWM Output: ";
				cout << "Right Winch: " << winch_right_cmd << " Left Winch: " << winch_left_cmd << endl;

				cout << "Euler Angles (Mahony): " << dt <<endl;
				cout << "MPU9250: Roll: " << roll_mpu_mahony << " Pitch: " << pitch_mpu_mahony << " Yaw: " << yaw_mpu_mahony << endl;
				cout << "LSM9DS1: Roll: " << roll_lsm_mahony << " Pitch: " << pitch_lsm_mahony << " Yaw: " << yaw_lsm_mahony << endl;
				cout << "Euler Angles (Madwick): " << dt << endl;
				cout << "MPU9250: Roll: " << roll_mpu_madgwick << " Pitch: " << pitch_mpu_madgwick << " Yaw: " << yaw_mpu_madgwick << endl;
				cout << "LSM9DS1: Roll: " << roll_lsm_madgwick << " Pitch: " << pitch_lsm_madgwick << " Yaw: " << yaw_lsm_madgwick << endl;
				cout << "Integrated Yaw (fallback):" << endl;
				cout << "MPU9250: Yaw: " << yaw_mpu_integrated_degrees << " LSM9DS1: " << yaw_lsm_integrated_degrees << endl;

				cout << "Controller Variables: ";
				cout << "\tSystem ID, A: " << A << " B: " << B << " C: " << C;
				cout << "\tNDI, wn: " << wn << " zeta: " << zeta;
				cout << "\tPID, kp: " << kp << " ki: " << ki << " kd: " << kd << endl;
				cout << "Yaw Desired: " << yaw_desired << " Yaw Error: " << yaw_error << " Yaw Error Rate: " << yaw_error_rate << " Yaw Error Sum: " << yaw_error_sum << endl;
				multisine_counter++;

			}

//dbmsg_local = true;
			if(dbmsg_global || dbmsg_local)
			{
				// Alternate debug message, just print out the most current duration for each loop occasionally
				cout << "Printing the most current expected vs. actual time since last execution for each loop" << endl;
				for(int i = 0 ; i < NUM_LOOPS ; i++)
				{
					cout << "For " << frequency[i] << "(Hz) loop, expected: " << duration[i] << "(us), actual: " << watcher[i] << endl;
				}
			}
//dbmsg_local = false;

			// Debug message, these slow down execution considerably
			// with messages enabled this loop may not operate at the expected frequency
			if(dbmsg_global || dbmsg_local) {cout << "This is the " << frequency[8] << "(Hz) loop." << endl
				<< "Expected time since last execution " << duration[8]
				<< "(us), Actual time since last execution " << time_now-timer[8] << "(us)" << endl;}
			watcher[8] = time_now - timer[8]; // used to check loop frequency
			timer[8] = time_now;
		}

	}

	// tidy up and close the file when we exit the inner while loop
	//fout.close();
	}
	return 0;
}
