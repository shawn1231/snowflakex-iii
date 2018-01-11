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
using namespace std;

const bool dbmsg_global = false; // set flag to display all debug messages
bool dbmsg_local  = false; // change in the code at a specific location to view local messages only

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
const float output_range1[2] = {0,100};
const float output_range2[2] = {0,200};
const float output_range3[2] = {-100,100};

//---------------------------------------------------------------------------------------------------------------IMU Declarations
// vars to hold mpu values
float a_mpu[3];
float g_mpu[3];
float m_mpu[3];
// vars to hold lsm values
float a_lsm[3];
float g_lsm[3];
float m_lsm[3];

//-------------------------------------------------------------------------------------------------------------Servo Declarations
#define WINCH_RIGHT 1 // right hand winch servo, 1 is the servo rail position
#define WINCH_LEFT 0 // left hand winch servo, 0 is the servo rail position
#define MAX_DEFLECTION 0.500
#define LINE_NEUTRAL 1.500
float winch_right_cmd = 0;
float winch_left_cmd = 0;

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
	//-------------------------------------------------------------------------------------------Loop timing (scheduler) Initialization
	// initialize the time variables
	gettimeofday(&time_obj, NULL);
	tse          = time_obj.tv_sec*1000000LL + time_obj.tv_usec; // time since epoch (us)
	//tse        = time_obj.tv_sec*1000LL + time_obj.tv_usec/1000; // time since epoch (ms)
	time_start = tse; // time since epoch (ms), will be used to calculate time since start (beginning at 0us)

	// declare the scheduler frequencies
	cout << endl;
	cout << "Establishing loop frequencies.........." << endl;
	const int frequency [NUM_LOOPS] = {500,1,1,100,50,20,10,5,1}; //Hz
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
	cout << "Currently using " << rcinput.channel_count << "channels......." << endl;
	cout << " --RC Input successfully initialized-- " << endl;
	cout << "Creating RC Input range coefficients   " << endl;
	float *coefficients1 = calculate_coefficients(input_range,output_range1);
	float *coefficients2 = calculate_coefficients(input_range,output_range2);
	float *coefficients3 = calculate_coefficients(input_range,output_range3);
	cout << "Succesffuly created range coefficnents " << endl;
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
	//------------------------------------------------------------------------------------------------------------------Welcome Message
	usleep(1000000);
	cout << endl;
	cout << "======================================" << endl;
	cout << "Initialization completed.............." << endl;
	cout << "======================================" << endl << endl;
	usleep(1000000);
	cout << "======================================" << endl;
	cout << "Main Loop starting now................" << endl;
	cout << "======================================" << endl << endl;
	usleep(1000000);


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
			usleep(5000);
			standby_message_timer++;}
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
		filename_str = file_location+filename_str;
		// used to put a number on the end of the file if it is a duplicate
		int filename_index = 1;
		// loop while the file name already exists or until we are out of indices to write to
		cout << "Setting up log file...................." << endl;
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
			filename_str = file_location+temp;
			filename_index++;
			usleep(50000);} // don't try to read the files at warp speed
		// create the file output stream object
		ofstream fout;
		// open the file
		cout << "Log file created at: " << endl << filename_str << endl;
		fout.open(filename_str,ios::out);
		// header string
		fout << "today,microseconds_since_start,"
			"msl,rc0,rc1,rc2,rc3,rc4,rc5,"
			"a_mpu[0],a_mpu[1],a_mpu[2],a_lsm[0],a_lsm[1],a_lsm[2],"
			"g_mpu[0],g_mpu[1],g_mpu[2],g_lsm[0],g_lsm[1],g_lsm[2],"
			"m_mpu[0],m_mpu[1],m_mpu[2],m_lsm[0],m_lsm[1],m_lsm[2],"
			"winch_right_cmd,winch_left_cmd" << endl;

	while(rc_array[5]>1500)
	{
		// refresh time now to prepare for another loop execution
		gettimeofday(&time_obj, NULL); // must first update the time_obj
		//tse        = time_obj.tv_sec*1000LL + time_obj.tv_usec/1000; // update tse (ms)
		tse        = time_obj.tv_sec*1000000LL + time_obj.tv_usec; // update tse (us)
		time_now   = tse - time_start; // calculate the time since execution start by subtracting off tse

		if( (time_now-timer[0]) > duration[0])
		{

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
//			rc_array_scaled[0] = scale_output(rc_array[0],coefficients1);
//			rc_array_scaled[1] = scale_output(rc_array[1],coefficients1);
//			rc_array_scaled[2] = scale_output(rc_array[2],coefficients1);
//			rc_array_scaled[3] = scale_output(rc_array[3],coefficients2);
//			rc_array_scaled[4] = scale_output(rc_array[4],coefficients3);
			for(int i = 0 ; i < rcinput.channel_count ; i++ )
			{
				rc_array_scaled[i] = scale_output(rc_array[i],coefficients1);
			}



			//-----------------------------------------------------------------------------------------------------------------PWM Output
			// these winch positions represent minimum line lengths (maximum deflection)
			winch_right_cmd = LINE_NEUTRAL-MAX_DEFLECTION;
			winch_left_cmd = LINE_NEUTRAL+MAX_DEFLECTION;

			pwm_out.set_duty_cycle(WINCH_RIGHT, winch_right_cmd);
			pwm_out.set_duty_cycle(WINCH_LEFT, winch_left_cmd);



			//-------------------------------------------------------------------------------------------------------Data Log File Output
			fout << today << "," << time_now << "," << msl << ",";
			for(int i = 0 ; i < rcinput.channel_count ; i++){
				fout << rc_array[i] << ",";}
			fout << a_mpu[0] << "," << a_mpu[1] << "," << a_mpu[2] << "," << a_lsm[0] << "," << a_lsm[1] << ",";
			fout << a_lsm[2] << "," << g_mpu[0] << "," << g_mpu[1] << "," << g_mpu[2] << "," << g_lsm[0] << "," << g_lsm[1] << ",";
			fout << g_lsm[2] << "," << m_mpu[0] << "," << m_mpu[1] << "," << m_mpu[2] << "," << m_lsm[0] << "," << m_lsm[1] << "," << m_lsm[2];
			fout << winch_right_cmd << "," << winch_left_cmd << endl;



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
					cout << " channel " << i << ": " << rc_array[i]*coefficients[0] + coefficients[1];
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
