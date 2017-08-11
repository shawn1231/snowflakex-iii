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
#include <iomanip> // used to force GPS data to output with correct precision
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
// ADC Includes
#include <Navio/ADC.h>
// GPS Includes
#include "Navio/Ublox.h"
// Threading Includes
#include <mutex>
#include <thread>

using namespace std;

//---------------------------------------------------------------------------------------------------User Configurable Parameters
const bool dbmsg_global = false; // set flag to display all debug messages
bool dbmsg_local  = false; // change in the code at a specific location to view local messages only
char control_type = 'x'; // valid options:  p=PID , n=NDI , g=glide (no spin), m=multisine , s=input sweep, c=rc control
char heading_type = '1'; // valid otpoins 1=N, 2=E, 3=S, 4=W, u=user, c=rc control, n=navigation algorithm
float user_heading = 115; //degress, only used if us is the heading type
bool live_gains = false;


//------------------------------------------------------------------------------------------------------------System ID Variables
float A           = -0.7391;
float B           = -43.362;
float C           = 17.417;
float wn_start    = .5;
float zeta_start  = 2;
float wn          = 0;
float zeta        = 0;
float ki_ndi_start = .005;

//-------------------------------------------------------------------------------------------------------Common Control Variables
float yaw_desired    = 0;
string heading_type_message = "null";
float yaw_error      = 0 , yaw_error_previous = 0;
float yaw_error_sum  = 0;
float yaw_error_rate = 0;
int num_wraps = 0;
float yaw_prev = 0;

//------------------------------------------------------------------------------------------------------------------PID Variables
// 8:15 gains - starting point
//float kp_start    = -.03246;
//float ki_start    = -.03494;
//float kd_start    = -.007003;

//small chute gains right actuator
//float kp_start = .32068;
//float ki_start = .22595;
//float kd_start = .11378;

//gains from pid-tuning-6-jun-20-05-05-0
//double kp_start = .1950;
//double ki_start = .0027;
//double kd_start = .0600;

// Control Validation Program - Run 1, Baseline Gains
//double kp_start = .15874;
//double ki_start = .11182;
//double kd_start = .05628;

// Roberts half gains
double kp_start = .07937;
double ki_start = .05591;
double kd_start = .02814;

// Control Validation Program - Run 2, OK Gains
//double kp_start = .2686;
//double ki_start = .1143;
//double kd_start = .1086;

// Control Validation Program - Run 3, Vigorous Gains
//double kp_start = .314;
//double ki_start = .114;
//double kd_start = .114;

string control_type_message = "null";

//---------------------------------------------------------------------------------------------------------------Step Input Timer
int step_counter;
#define NUM_STEPS 10
#define STEP_SIZE 0.06
#define INITIAL_DEFLECTION .3

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
RCInput rcinput{}; const float input_range[2] = {1088,1940}; // range is the same for all channels

// for PID tuning
const float output_range[6][2] = {{-.05,.30},{2,-2},{-.185,.500},{-180,180},{-.1,.1},{-.1,.1}};
float coefficients[6][2];

//---------------------------------------------------------------------------------------------------------------IMU Declarations
#define DECLINATION 12.71 //magnetic declination for camp roberts
#define WRAP_THRESHOLD 160.00 // wrap threshold for wrap counter (yaw is +/-180, need to make it continuous
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
#define LINE_OFFSET .30
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

//Campus Quad Offsets
float mag_offset_mpu[3] = {19.236,7.985,59.101};
float mag_rotation_mpu[3][3] = {{.769,-.166,.063},{-.166,.725,.075},{.063,.075,.976}};
float mag_offset_lsm[3] = {-5.388,1.707,72.647};
float mag_rotation_lsm[3][3] = {{.843,-.192,.088},{-.192,.677,.112},{.088,.112,.949}};

//---------------------------------------------------------------------------------------------------------------GPS Declarations
double time_gps = 0;
double lat = 0;
double lng = 0;
double alt_ellipsoid = 0;
double msl_gps = 10000;
double horz_accuracy = 0;
double vert_accuracy = 0;
int status_gps = 0x00; // default condition - no fix
string status_gps_string = "no fix";

//--------------------------------------------------------------------------------------------------------Threading Decalarations
bool buffer_full = false;
bool buffer_empty = false;
mutex mtx;

//----------------------------------------------------------------------------------------------------------Waypoint Declarations
double waypoints[50][3]; // waypoint array is 50x3
double target[2] = {35.7178528,-120.76411}; // from step input payload drop

//-----------------------------------------------------------------------------------------------------------Logfile Declarations
// these are used to format the filename string
#define FILENAME_LENGTH 12
#define FILENAME_OFFSET 4
// leave extra room for '.csv' and potential '-1', '-2', etc.
char filename[FILENAME_LENGTH+6];
// log file location (relative path)
string file_location = "/home/pi/Navio2/C++/Examples/NewCode/LogFiles/";
// function to check if file exists so that the filename can be dynamically changed
bool file_exists(string name_of_file){
	ifstream checkfile(name_of_file); // try to read from the file
	return checkfile;} // cast the result to bool, if file opens, this returns true (it exists)


// Define Threading Function
void gpt_thread(){
	if(false){ // this will be replaced with the gps read function
		buffer_empty = false;
		mtx.lock();
		// write gps data from private function to local variables
		mtx.unlock();
		buffer_full = true;
	}
}


int main( int argc , char *argv[])
{
	//----------------------------------------------------------------------------------------------------------------File Title Prefix
	double kp = kp_start*.0175; // convert all 3 to radians
	double ki = ki_start*.0175;
	double kd = kd_start*.0175;
	float ki_ndi = ki_ndi_start*.0175; // convert to radians
	int multisine_counter = 0;
	int parameter; // parameter is the argument passed in with the program call, create it here
	char *logfile_prefix; // pointer to a character array which will contain the log_file prefix when using the -d parameter
	string logfile_prefix_string; // previous variable will be converted to a string before using it to make a file
	logfile_prefix_string = file_location; // used to store the path
	string logfile_prefix_fromfile; // used to store the prefix when using the -f parameter
	ifstream ifs; // create a new file stream object, this will be reused for each file we need to read
	while((parameter = getopt(argc,argv, "hfd:")) != -1){ // read in the program options
		switch(parameter){ // switch on the character after the dash example:  sudo ./NewCode -d <filename>"
			case 'h' : // h parameter calls the help message
				cout << "Use option \"-d <FilePrefix>\" to insert a prefix into the filename" << endl;
				cout << "Use option \"-f\" to read configuration from file (configuration.csv)" << endl;
				return EXIT_FAILURE; // do not continue executing after displaying the help message
				break;
			case 'f' : // f parameter reads important variables from a configutation.txt file
				cout << "Reading user configuration from file" << endl;
				ifs.open("configuration.txt"); // open the configuration file
				ifs >> control_type; // read in the control type (character)
				cout << "Control type: " << control_type << endl; // echo to console
				ifs >> heading_type; // read in the heading type (character)
				cout << "Heading type: " << heading_type << endl; // echo to console
				ifs >> user_heading; // read in the heading (number), only used for user heading
				cout << "User heading: " << user_heading << " degrees" << endl; // echo to the console
				getline(ifs,logfile_prefix_fromfile); // this is a hack to get to the next line
				getline(ifs,logfile_prefix_fromfile); // read in the prefix from the configuration file
				logfile_prefix_string = file_location+logfile_prefix_fromfile+'_'; // stick an underscore after the file prefix
				cout << "Logfile prefix: " << logfile_prefix_fromfile << endl; // echo to the console
				ifs.close(); // close the file
				break;
			// this option reads in a prefix from the program call and uses the defaults for everything else
			case 'd' : logfile_prefix = optarg; logfile_prefix_string = file_location+string(logfile_prefix)+'_'; break;
			// stop execution if option flag is invalid
			case '?' : cout << "Invalid option.  Use -h for help" << endl; return EXIT_FAILURE;}}


	cout << endl;
	cout << "=======================================" << endl;
	cout << "         Begin Initialiation           " << endl;
	cout << "=======================================" << endl;
	cout << endl;
	cout << "Reading mag calibration from file......" << endl;
	ifs.open ("/home/pi/Navio2/C++/Examples/NewCode/mpu_mag_cal.csv"); // name of the mpu mag calibration file
	if(ifs){ // if successful, read the values and print a message
		cout << "MPU9250 offsets:" << endl;
	for(int i = 0 ; i < 3 ; i++ ){ // colum iterator, offsets are the first row (1x3) inside the 4x3 data file
		if(i != 0){ // on the first and last iteration there is no comma to catch
			cout << ", "; // print a comma to make output readable
			char delim; // create a character inside this scope, it is just a dummy variable
			ifs >> delim;} // stream the comma to the dummy character so it can be discarded
		ifs >> mag_offset_mpu[i]; // stream the offsets into the correct position in the offset array
		cout << mag_offset_mpu[i];} // output the offset array to the console
	cout << endl; // end line, prepare to read/write the rotation matrix
	cout << "MPU9250 rotation matrix:" << endl;
	for(int i = 0 ; i < 3 ; i++ ){ // row iterator, matrix is 3x3
		for(int j = 0 ; j < 3 ; j++ ){ // column iterator
			if(j != 0){ // on the first and last iteration there is no comma to catch
				cout << ", "; // print a comma to make the output readable
				char delim; // create a character inside this scope, it is just a dummy variable
				ifs >> delim;} // stream the comma to the dummy character so it can be discarded
			ifs >> mag_rotation_mpu[i][j]; // stream the rotation matrix values to the correct positions in the array
			cout << mag_rotation_mpu[i][j];} // output the rotation matrix to the console
		cout << endl;}}
	else{ // if the file open operation is not successful
	cout << "Cannot read MPU offsets, using defaults" << endl;} // print that the file could not be read, this is not a fatal error
	ifs.close(); // close the current file to prepare for the next file read operation

// ---not going to duplicate comments, this next section is just the same as above except for a different sensor
	ifs.open("/home/pi/Navio2/C++/Examples/NewCode/lsm_mag_cal.csv");
	if(ifs){
		cout << "LSM9DS1 offsets:" << endl;
		for(int i = 0 ; i < 3 ; i++ ){
			if(i != 0){
				cout << ", ";
				char delim;
				ifs >> delim;}
			ifs >> mag_offset_lsm[i];
			cout << mag_offset_lsm[i];}
		cout << endl;
		cout << "LSM9DS1 rotation matrix:" << endl;
		for(int i = 0 ; i < 3 ; i++ ){
			for(int j = 0 ; j < 3 ; j++ ){
				if(j != 0){
					cout << ", ";
					char delim;
					ifs >> delim;}
				ifs >> mag_rotation_lsm[i][j];
				cout << mag_rotation_lsm[i][j];}
			if(i < 2){
				cout << endl;}}}
	else{
		cout << "Cannot read LSM offsets, using defaults";}
	ifs.close();

	// propogate the waypoints matrix with 0's, the value 0 will be used to mark the end of valid data
	for(int i = 0 ; i < 50 ; i++ ){
		for(int j = 0 ; j < 3 ; j++){
			//cout << "we made it to here" << endl;
			waypoints[i][j] = 0;}} // fill with zeros, this will mark the EOF

	// read in the waypoints file
	ifs.open("/home/pi/Navio2/C++/Examples/NewCode/waypoints.csv");
	if(ifs){
		cout << "Reading waypoints from file............" << endl;
		cout << "lat\tlong\talt" << endl;
		for(int i = 0 ; i < 50 ; i++ ){ // row iterator
			for(int j = 0 ; j < 3 ; j++){ // column iterator
				if(j!=0){
					if(waypoints[i][j] != 0){
						cout << ",";}
					char delim;
					ifs >> delim;}
				ifs >> waypoints[i][j];
				}

			}
		for(int i = 0 ; i < 50 ; i++){
			for(int j = 0 ; j < 3 ; j++){
				if(waypoints[i][0]!=0){
					cout << waypoints[i][j];
					if(j!=2){
						cout << ",";}}}
				if(waypoints[i+1][0] != 0){
					cout << endl;}}}

	else{
		cout << "Could not read waypoints" << endl;
		if(heading_type = 'n'){
			cout << "Halting execution, cannot navigate without waypoints" << endl;
			// end execution if a navigation heading is being used but there are not waypoints
			return EXIT_FAILURE;}
	}



	//-------------------------------------------------------------------------------------------Loop timing (scheduler) Initialization
	// initialize the time variables
	gettimeofday(&time_obj, NULL); // standard unix function to get the current time since epoch
	tse          = time_obj.tv_sec*1000000LL + time_obj.tv_usec; // time since epoch (converted to microseconds)
	time_start = tse; // time since epoch (ms), will be used to calculate time since start (beginning at 0us)

	// declare the scheduler frequencies
	cout << endl;
	cout << "Establishing loop frequencies.........." << endl;
	// 8-4-2017 changed all timer related variables to long long, timer variable (int) was overflowing and causing erratic timing
	// if code had been running for ~35 minutes, tested after fixing and overflow is no longer occuring after 35 minutes
	const float frequency [NUM_LOOPS] = {300,1000,1,100,50,20,10,.33,1}; //Hz
	unsigned long long duration [NUM_LOOPS]; // stores the expected time since last execution for a given loop
	unsigned long long timer [NUM_LOOPS]; // stores the time since the last execution in a given loop
	unsigned long long watcher [NUM_LOOPS]; // used for monitoring actual timer loop durations

	// calculate the scheduler us delay durations and populate the timer array with zeros
	cout << "Calculating loop delay durations......." << endl;
	cout << "Populating loop timers................." << endl;
	for(int i = 0 ; i < NUM_LOOPS ; i++){ // populate the durations using the passed values for frequency, echo everything to the console
		duration[i] = 1000000/frequency[i];
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
	cout << "Currently using " << rcinput.channel_count << " channels............." << endl;
	cout << " --RC Input successfully initialized-- " << endl;
	cout << "Creating RC Input range coefficients   " << endl;
	for(int i = 0 ; i < rcinput.channel_count ; i++){ // using a custom class for converting bounded values to values in a new range
		// in the Python version of this function, both parameters are returned by one function
		// since multiple values may not be returned by a single function this version uses a function for each scaling parameter
		coefficients[i][0] = calculate_slope(input_range,output_range[i]); // first call the slope calculator
		coefficients[i][1] = calculate_intercept(input_range,output_range[i],coefficients[i][0]); // next calculate the intercept
	}
	cout << "Successfully created range coefficients" << endl;

	//---------------------------------------------------------------------------------------------------------------IMU Initialization
	cout << "Initializing IMUs......................" << endl;
	InertialSensor *mpu;
	mpu = new MPU9250();
	mpu->initialize();
	cout << " --MPU9250 successfully initialized--" << endl;
	InertialSensor *lsm;
	lsm = new LSM9DS1();
	lsm->initialize();
	cout << " --LSM9DS1 successfully initialized--" << endl;
	cout << "Populating IMU sensor arrays..........." << endl;
	for(int i = 0; i < 3 ; i++){ // seed everything with a value of 0
		a_mpu[i]=g_mpu[i]=m_mpu[i]=a_lsm[i]=g_lsm[i]=m_lsm[i] = 0.0;}
	//-------------------------------------------------------------------------------------------------------------Servo Initialization
	cout << "Initializing PWM Output................" << endl;
	PWM pwm_out;
	// create pwm output object and initialize right and left winch servos, stop execution if servos cannot be initialized
	// NOTE!!! if the code is erroring out here, the first thing to check is to make sure that you are are running the code with sudo
	if(!pwm_out.init(WINCH_RIGHT)){ // right hand side winch servo
		cout << "Cannot Initialize East Side Winch Servo" << endl;
		cout << "Make sure you are root" << endl;
		return EXIT_FAILURE;}
	if(!pwm_out.init(WINCH_LEFT)){ // left hand side winch servo
		cout << "Cannot Initialize West Side Winch Servo" << endl;
		cout << "Make sure you are root" << endl;
		return EXIT_FAILURE;}
	cout << "Enabling PWM output channels..........." << endl;
	pwm_out.enable(WINCH_RIGHT); // both init() and enable() must be called to use the PWM output on a particular pin
	pwm_out.enable(WINCH_LEFT);
	cout << "Setting PWM period for 50Hz............" << endl;
	pwm_out.set_period(WINCH_RIGHT , 50); // set the PWM frequency to 50Hz
	pwm_out.set_period(WINCH_LEFT , 50);
	cout << "  --PWM Output successfully enabled-- " << endl;
	//--------------------------------------------------------------------------------------------------------------ADC Initialization
	cout << "Initializing ADC......................." << endl;
	ADC adc{};
	adc.init();
	float adc_array[adc.get_channel_count()] = {0.0f};
	for(int i = 0 ; i < ARRAY_SIZE(adc_array) ; i++){
		adc_array[i] = adc.read(i);}
	cout << "     --ADC successfully enabled--     " << endl;

	//-------------------------------------------------------------------------------------------------------------AHRS Initialization
	cout << "Initializing gyroscope................." << endl;
	cout << "Reading gyroscope offsets.............." << endl;
	// gyroscope offsets generated by sampling the gyroscope 100 times when the program executes, averaging the 100 samples
	// and then subtracting off the newly acquired offset values evertime the gyro is read later on in the code
	for(int i = 0 ; i < 100 ; i++){
		// read both sensors, we are only concerned with gyroscope information right now
		mpu->update(); // update the mpu sensor
		mpu->read_gyroscope(&g_mpu[0],&g_mpu[1],&g_mpu[2]); // read the updated info
		lsm->update(); // update the lsm sensor
		lsm->read_gyroscope(&g_lsm[0],&g_lsm[1],&g_lsm[2]); // read the updated info
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
		usleep(10000);}
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
	// finally write the acquired offsets to the ahrs objects which have a member function which automatically handles
	// the application of offsets at each update/read cycle
	ahrs_mpu_mahony.setGyroOffset(offset_mpu[0],offset_mpu[1],offset_mpu[2]);
	ahrs_lsm_mahony.setGyroOffset(offset_lsm[0],offset_lsm[1],offset_lsm[2]);
	ahrs_mpu_madgwick.setGyroOffset(offset_mpu[0],offset_mpu[1],offset_mpu[2]);
	ahrs_lsm_madgwick.setGyroOffset(offset_lsm[0],offset_lsm[1],offset_lsm[2]);
	cout << " --Gyroscope offsets stored in AHRS--  " << endl;
	cout << "Setting magnetometer calibration......." << endl;
	ahrs_mpu_mahony.setMagCalibration(mag_offset_mpu,mag_rotation_mpu);
	ahrs_lsm_mahony.setMagCalibration(mag_offset_lsm,mag_rotation_lsm);
	ahrs_mpu_madgwick.setMagCalibration(mag_offset_mpu,mag_rotation_mpu);
	ahrs_lsm_madgwick.setMagCalibration(mag_offset_lsm,mag_rotation_lsm);
	cout << "--Magnetometer offsets stored in AHRS--" << endl;
	cout << "-Magnetometer rotations stored in AHRS-" << endl;

	//---------------------------------------------------------------------------------------------------------------GPS Initialization
	vector<double> pos_data; // this vector will contain undecoded gps information
	int wind_level_index = 0; // this is the counter for the current altitude level used in the navigation algorithm
	Ublox gps;
	cout << "Initializing GPS......................." << endl;
	if(!gps.testConnection()){ // check if the gps is working
		cout << "    --ERROR, GPS not initialized--    " << endl;
		if(heading_type == 'n'){
			// stop program executiong if gps won't initialize and navigation heading type is selected
			cout << "Fatal exception, navigation impossible" << endl;
			cout << "without GPS, try restarting..........." << endl;
			return EXIT_FAILURE;}}
	else{ // gps is good!
		cout << "  --GPS successfully initialized--    " << endl;}

	//------------------------------------------------------------------------------------------------------------------Welcome Message
	usleep(500000);
	cout << endl;
	cout << "=======================================" << endl;
	cout << "Initialization completed..............." << endl;
	cout << "=======================================" << endl << endl;
	usleep(500000);
	cout << "=======================================" << endl;
	cout << "Main Loop starting now................." << endl;
	cout << "=======================================" << endl << endl;
	usleep(500000);

	//----------------------------------------------------------------------------------------------------------Log File Initialization
	while(true)
	{
		int standby_message_timer = 0; // used to limit the frequency of the standby message
		while(!((rc_array[5]>1500)&&(adc_array[4]<4000))){
//bool temp_flag = false; // uncomment for testing with no transmitter
//while(!temp_flag){ // uncomment for testing with no transmitter
			if(standby_message_timer > 250){
				cout << endl << "---------------------------------------" << endl << "           Autopilot Inactive         " << endl;
				cout << "  Dynamic Lines at Neutral Deflection " << endl;
				cout << "         Waiting for Killswitch       " << endl << "--------------------------------------" << endl;
				standby_message_timer = 0;}
			// when the autopilot is inactive, set both winches to the neutral deflection
			pwm_out.set_duty_cycle(WINCH_RIGHT, LINE_NEUTRAL);
			pwm_out.set_duty_cycle(WINCH_LEFT, LINE_NEUTRAL);
			// since this loop executes based on an rc and an adc condition, we have to poll these devices for new status
			rc_array[5] = rcinput.read(5);
			adc_array[4] = adc.read(4);
			// step counter needs to be set to zero also, this is a hack because the numbering is messed up in the code
			// we start at negative one here and presumably the counter is incremented before 1st execution
			step_counter = -1;
			usleep(5000);
			standby_message_timer++; // increment the message delay timer
			// everything that needs to be set to zero by the killswitch goes here
			multisine_counter = 0; // time counter for the multisine input
			yaw_mpu_integrated = 0; // integrated yaw
			yaw_mpu_integrated_previous = 0;
			yaw_lsm_integrated = 0;
			yaw_lsm_integrated_previous = 0;
			wind_level_index = 0; // wind level for navigation, can only be incremented by the main loop to avoid "waypoint indecision"
			for(int i = 0 ; i < 3 ; i++){
				gyro_z_lsm_old[i] = 0;
				gyro_z_mpu_old[i] = 0;
			}
			yaw_error_sum       = 0; // prevent integral wind up
			yaw_error           = 0;
			yaw_error_previous  = 0;
//temp_flag = true; // uncomment for testing with no transmitter

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
			"winch_right_cmd,winch_left_cmd,yaw_integrated_mpu,yaw_integrated_lsm,"
			"A,B,C,wn,zeta,kp,ki,kd,"
			"yaw_desired,yaw_error,yaw_error_previous,yaw_error_rate,yaw_error_sum,adc_array[5],control_type,heading_type,"
			"time_gps,lat,lng,alt_ellipsoid,msl_gps,horz_accuracy,vert_accuracy,"
			"status_gps" << endl;
		usleep(20000);
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
		num_wraps = 0;
		yaw_prev = 0;

	while((rc_array[5]>1500)&&(adc_array[4]<4000))
//while(true)
	{
		// refresh time now to prepare for another loop execution
		gettimeofday(&time_obj, NULL); // must first update the time_obj
		tse        = time_obj.tv_sec*1000000LL + time_obj.tv_usec; // update tse (us)
		//tse        = tse + 2000000000; // uncomment to test integer overflow fix
		time_now   = tse - time_start; // calculate the time since execution start by subtracting off tse

		if( (time_now-timer[0]) > duration[0])
		{
			//----------------------------------------------------------------------------------------------------------------AHRS Update
			dt = time_now-timer[0];
			dt = dt/1000000.0; // convert from useconds

			if(waypoints[wind_level_index][0] != 0){
				if(msl_gps < waypoints[wind_level_index][2]){
					wind_level_index++;}}

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
				g_mpu_ahrs[i] = g_mpu[i]*(180/PI);
				a_lsm_ahrs[i] = a_lsm[i]/G_SI;
				g_lsm_ahrs[i] = g_lsm[i]*(180/PI);}

			ahrs_mpu_mahony.updateMahony(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0]*0.0175,g_mpu_ahrs[1]*0.0175,g_mpu_ahrs[2]*0.0175,m_mpu[0],m_mpu[1],m_mpu[2],dt);
			ahrs_lsm_mahony.updateMahony(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0]*0.0175,g_lsm_ahrs[1]*0.0175,g_lsm_ahrs[2]*0.0175,m_lsm[0],m_lsm[1],m_lsm[2],dt);
			ahrs_mpu_madgwick.updateMadgwick(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0]*0.0175,g_mpu_ahrs[1]*0.0175,g_mpu_ahrs[2]*0.0175,m_mpu[0],m_mpu[1],m_mpu[2],dt);
			ahrs_lsm_madgwick.updateMadgwick(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0]*0.0175,g_lsm_ahrs[1]*0.0175,g_lsm_ahrs[2]*0.0175,m_lsm[0],m_lsm[1],m_lsm[2],dt);
			//uncomment for non-global framed ahrs (no magnetometer)
//			ahrs_mpu_mahony.updateMahonyIMU(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0]*0.0175,g_mpu_ahrs[1]*0.0175,g_mpu_ahrs[2]*0.0175,dt);
//			ahrs_lsm_mahony.updateMahonyIMU(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0]*0.0175,g_lsm_ahrs[1]*0.0175,g_lsm_ahrs[2]*0.0175,dt);
//			ahrs_mpu_madgwick.updateMadgwickIMU(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0]*0.0175,g_mpu_ahrs[1]*0.0175,g_mpu_ahrs[2]*0.0175,dt);
//			ahrs_lsm_madgwick.updateMadgwickIMU(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0]*0.0175,g_lsm_ahrs[1]*0.0175,g_lsm_ahrs[2]*0.0175,dt);

			// update the que holding the old gyro values with the new gyro information
			gyro_z_mpu_old[2] = gyro_z_mpu_old[1];
			gyro_z_mpu_old[1] = gyro_z_mpu_old[0];
			gyro_z_mpu_old[0] = g_mpu_ahrs[2]-offset_mpu[2];
			gyro_z_lsm_old[2] = gyro_z_lsm_old[1];
			gyro_z_lsm_old[1] = gyro_z_lsm_old[0];
			gyro_z_lsm_old[0] = g_lsm_ahrs[2]-offset_lsm[2];
			// update previous integrated yaw variables
			yaw_mpu_integrated_previous = yaw_mpu_integrated;
			yaw_lsm_integrated_previous = yaw_lsm_integrated;

			//update the integrated gyro yaw (this is a fallback in case the heading is bad)
			yaw_mpu_integrated = (((gyro_z_mpu_old[0]+gyro_z_mpu_old[1])/2)*dt) + yaw_mpu_integrated_previous;
			yaw_mpu_integrated_degrees = yaw_mpu_integrated * (180/PI);
			yaw_lsm_integrated = (((gyro_z_lsm_old[0]+gyro_z_lsm_old[1])/2)*dt) + yaw_lsm_integrated_previous;
			yaw_lsm_integrated_degrees = yaw_lsm_integrated * (180/PI);

			watcher[0] = time_now - timer[0]; // used to check loop frequency
			timer[0] = time_now;
		}

		if( (time_now-timer[1]) > duration[1])
		{
			if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
			{
				// examples provided with Navio2 have gps code which is designed to run in a threaded program, the gps update
				// functions block execution, for this code, the functions have been modified, the gps is polled here at 1000hz
				// but valid new data only comes through occasionally, the NAV_STATUS message has been disabled entireley because
				// it was causing .75 second interruptions in the timer loop, in the future threading will be utilized for the
				// gps (then the imu and ahrs may be added to separate threads as well)
	                	time_gps = pos_data[0]/1000.00000;
		       	        lng = pos_data[1]/10000000.00000;
        	       		lat = pos_data[2]/10000000.00000;
	        	        alt_ellipsoid = (pos_data[3]/1000.00000)*3.28;
		               	msl_gps = (pos_data[4]/1000.00000)*3.28;
				//msl_gps = 2500-step_counter*160; //uncomment to test navigation algorithm
        		        horz_accuracy = (pos_data[5]/1000.00000)*3.28;
	                	vert_accuracy = (pos_data[6]/1000.00000)*3.28;
			}
			watcher[1] = time_now - timer[1]; // used to check loop frequency
			timer[1] = time_now;
			}

		if( (time_now-timer[2]) > duration[2])
		{
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
				rc_array_scaled[i] = scale_output(coefficients[i],float(rc_array[i]));
			}

			//-----------------------------------------------------------------------------------------------------------------ADC Update
			for(int i = 0 ; i < ARRAY_SIZE(adc_array) ; i++){
				adc_array[i] = adc.read(i);}

			//-----------------------------------------------------------------------------------------------------------------PWM Output
			// these are kept here for reference purposes only, the winch commands are further down inside this same loop now
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

			// determine what type of heading we are going to used based on the configuration file (or default parameter in the code)
			switch(heading_type){
				case '1':
					yaw_desired = 0; // North heading
					heading_type_message = "Fixed - North";
					break;
				case '2':
					yaw_desired = 90; // East heading
					heading_type_message = "Fixed - East";
					break;
				case '3':
					yaw_desired = 180; // South heading
					heading_type_message = "Fixed - South";
					break;
				case '4':
					yaw_desired = -90; // West heading
					heading_type_message = "Fixed - West";
					break;
				case 'u':
					yaw_desired = user_heading;
					heading_type_message = "User " + to_string(user_heading);
					break;
				case 'c':
					yaw_desired = rc_array_scaled[3]; // yaw stick
					heading_type_message = "RC " + to_string(rc_array_scaled[3]);
					yaw_desired = rc_array_scaled[3];
					break;
				case 's':
					heading_type_message = "Step Input (altitude driven)";
//					if(msl_gps > 1400){
//						yaw_desired = user_heading;}
//					if(msl_gps <= 1400){
//						yaw_desired = user_heading + 90;}

					if(msl_gps > 1700){
						yaw_desired = user_heading;}
					if(msl_gps <= 1700 && msl_gps > 1400){
						yaw_desired = user_heading + 90;}
					if(msl_gps <= 1400){
						yaw_desired = user_heading + 180;}

//					if(rc_array[2] > 1600){
//						yaw_desired = 90;
//					} else{
//						yaw_desired = 0; // set yaw desired here
					break;
				case 'n':
					heading_type_message = "Navigation Algorithm";
					//write code to determine heading here
					//lat_target
					//lon_target
					if(waypoints[wind_level_index][0] == 0)
					{
						yaw_desired = 100;
					}else{
						yaw_desired = atan2(lat-waypoints[wind_level_index][0],lng-waypoints[wind_level_index][1]);
					}
					break;
				case 'd':
					heading_type_message = "Dumb Navigation";
					yaw_desired = -atan2(10000*(lat-target[0]),10000*(lng-target[1]));
					yaw_desired = yaw_desired/.0157-90;
					break;
				default:
					heading_type_message = "Default - North";
					yaw_desired = 0;
					break;}

			// make yaw continuous instead of +/-180 using a wrap counter
			if(yaw_mpu_madgwick > WRAP_THRESHOLD && yaw_prev < - WRAP_THRESHOLD){
				num_wraps--;
			}
			if(yaw_mpu_madgwick < - WRAP_THRESHOLD && yaw_prev >  WRAP_THRESHOLD){
				num_wraps++;
			}

			// account for magnetic declination
			yaw_mpu_mahony = yaw_mpu_mahony + DECLINATION;
			yaw_lsm_mahony = yaw_lsm_mahony + DECLINATION;
			yaw_mpu_madgwick = yaw_mpu_madgwick + DECLINATION;
			yaw_lsm_madgwick = yaw_lsm_madgwick + DECLINATION;

			// calculate yaw error for controller
			yaw_prev              = yaw_mpu_madgwick;
			yaw_error_previous    = yaw_error;
			yaw_error             = yaw_desired -(yaw_mpu_madgwick+360*num_wraps);
			yaw_error_rate        = (yaw_error - yaw_error_previous)/dt_control;
			yaw_error_sum         = (((yaw_error + yaw_error_previous)*dt_control)/2) + yaw_error_sum;

			// saturate the yaw error sum as a reasonable value
			if(yaw_error_sum > 150){
				yaw_error_sum = 150;
			} if(yaw_error_sum < -150){
				yaw_error_sum = -150;
			}

			// hack to avoid changing the variable name "t" in the multisine
			float t = time_now*1e-6;

			// gains are modified live according to rc transmitter stick positions
			if(live_gains){
				kp    =  (kp_start+rc_array_scaled[2])*(.0175);
				ki    =  (ki_start+rc_array_scaled[3])*(.0175);
				kd    =  (kd_start+rc_array_scaled[0])*(.0175);
				wn    =  wn_start+rc_array_scaled[2];
				zeta  =  zeta_start+rc_array_scaled[1];
				ki_ndi = ki_ndi_start+rc_array_scaled[3];}
			else{
				kp = kp_start*.0175;
				ki = ki_start*.0175;
				kd = kd_start*.0175;
				wn = wn_start;
				zeta = zeta_start;
				ki_ndi = ki_ndi_start;}

			// control type is determined by configuration parameter from file (or default in code)
			switch(control_type){
				case 'p':
					control_type_message = "PID";
					if(rc_array[4] > 1500){
						winch_right_cmd = LINE_NEUTRAL + (kp*yaw_error + ki*yaw_error_sum + kd*yaw_error_rate);
						winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET;
					} else {
						winch_left_cmd = LINE_NEUTRAL + (kp*yaw_error + ki*yaw_error_sum + kd*yaw_error_rate);
						winch_right_cmd = LINE_NEUTRAL - LINE_OFFSET;}
					break;
				case 'n':
					control_type_message = "NDI";
					winch_right_cmd = LINE_NEUTRAL - (1/B)*(-2*zeta*wn*g_mpu[2]-A*g_mpu[2]+wn*wn*(yaw_desired-(yaw_mpu_madgwick+360*num_wraps))*.0175)-(C/B)+ki_ndi*yaw_error_sum;
					winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET;
					break;
				case 'r': // minimum deflection, for rigging
					control_type_message = "minimum deflection";
					winch_right_cmd = LINE_NEUTRAL + MAX_DEFLECTION;
					winch_left_cmd = LINE_NEUTRAL - MAX_DEFLECTION;
					break;
				case 'g': // glide, both lines pulled
					control_type_message = "glide, no spin";
					winch_right_cmd = LINE_NEUTRAL - LINE_OFFSET;
					winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET;
					break;
				case 's': // input sweep
					control_type_message = "input sweep";
					if(rc_array[4] > 1500){
						if(step_counter >= 0){
							winch_right_cmd = LINE_NEUTRAL+INITIAL_DEFLECTION - step_counter*STEP_SIZE;}
						else{
							winch_right_cmd = LINE_NEUTRAL;}
						winch_left_cmd = LINE_NEUTRAL+LINE_OFFSET;
					} else{
						if(step_counter>=0){
							winch_left_cmd = LINE_NEUTRAL-INITIAL_DEFLECTION + step_counter*STEP_SIZE;}
						else{
							winch_left_cmd = LINE_NEUTRAL;}
						winch_right_cmd = LINE_NEUTRAL-LINE_OFFSET;}
					break;
				case 'c': // rc control
					control_type_message = "RC control";
					//for RC control, throttle is left servo, elevator is right servo, down on the sticks is line pull
					winch_right_cmd = LINE_NEUTRAL + rc_array_scaled[2];
					//winch_left_cmd = LINE_NEUTRAL - rc_array_scaled[2];
					winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET;
					break;
				case 'm': // multisine
					control_type_message = "Multisine";
					// multisine or step input, only enable when switch D is in the 1 or 2 position
					if(rc_array[4] > 1500) // switch A 0 for left, 1 for right
					{
						if(multisine_counter < 61){
							// right winch is dynamic, left winch is static
							winch_right_cmd = (LINE_NEUTRAL-LINE_OFFSET-.04*(sin(.2094*t-0.6111)+sin(.4189*t-1.3593)+sin(0.8976*t-.8035)+sin(2.0944*t+2.6684)+sin(2.8274*t-1.2069)+sin(3.7699*t+2.8186)+sin(4.7124*t-.8956)+sin(5.3407*t+1.8869)+sin(6.2832*t+2.1591)));
							winch_left_cmd = LINE_NEUTRAL+LINE_OFFSET;}
						else{
							// after 60 seconds disable both and return to servo neutral
							winch_right_cmd = LINE_NEUTRAL;
							winch_left_cmd = LINE_NEUTRAL;}
					} else	{
						// apply the same things to the left hand side if the switch is down (0 position)
						if(multisine_counter < 61){
							winch_left_cmd = LINE_NEUTRAL+LINE_OFFSET+.04*(sin(.2094*t-0.6111)+sin(.4189*t-1.3593)+sin(0.8976*t-.8035)+sin(2.0944*t+2.6684)+sin(2.8274*t-1.2069)+sin(3.7699*t+2.8186)+sin(4.7124*t-.8956)+sin(5.3407*t+1.8869)+sin(6.2832*t+2.1591));
							winch_right_cmd = LINE_NEUTRAL-LINE_OFFSET;}
						else{
							winch_right_cmd = LINE_NEUTRAL;
							winch_left_cmd = LINE_NEUTRAL;}}
					break;
				default:
					// default is PWM neutral
					control_type_message = "Default, PWM neutral";
					winch_right_cmd = LINE_NEUTRAL;
					winch_left_cmd  = LINE_NEUTRAL;
					break;}

			// this is kept here for reference only
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

			// always write the duty cycle, change control type by changing the switch case parameter
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
			fout << winch_right_cmd << "," << winch_left_cmd << ","  << yaw_mpu_integrated_degrees << ",";
			fout << yaw_lsm_integrated_degrees << "," << A << "," << B << "," << C << ",";
			fout << wn << "," << zeta << "," << kp << "," << ki << "," << kd << ",";
			fout << yaw_desired << "," <<  yaw_error << "," << yaw_error_previous << "," << yaw_error_rate << ",";
			fout << yaw_error_sum << "," << adc_array[5] << "," << control_type << "," << heading_type << ",";
			fout << setprecision(9) << time_gps << "," << lat << "," << lng << "," << alt_ellipsoid << ",";
			fout << msl_gps << "," << horz_accuracy << "," << vert_accuracy << "," << status_gps << endl;

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

			watcher[4] = time_now - timer[4]; // used to check loop frequency
			timer[4] = time_now;
		}


		if( (time_now-timer[5]) > duration[5]){
			// unused timer loop
			watcher[5] = time_now - timer[5]; // used to check loop frequency
			timer[5] = time_now;}

		if( (time_now-timer[6]) > duration[6]){
			// unused timer loop
			watcher[6] = time_now - timer[6]; // used to check loop frequency
			timer[6] = time_now;}

		if( (time_now-timer[7]) > duration[7]){

			// this step counter is used for input sweeps (originally "step" inputs)
			step_counter++;
			if(step_counter>NUM_STEPS-1){
				step_counter = 0;}
			watcher[7] = time_now - timer[7]; // used to check loop frequency
			timer[7] = time_now;}

		if( (time_now-timer[8]) > duration[8]){
			//Output Message
			if(dbmsg_global || dbmsg_local){
			//if(!dbmsg_global && !dbmsg_local){
				cout << endl;
				cout << "Current filename: " << filename_str << endl;
				cout << "Barometer Altitude: " << msl << " ft" << endl;
				cout << "Raw RC Input:";
				for(int i = 0 ; i < rcinput.channel_count ; i++ ){
					cout << " channel " << i << ": " << rc_array[i];
					if(i != rcinput.channel_count -1){
						cout << ",";}}
				cout << endl;
				cout << "Scaled RC Input:";
				for(int i = 0 ; i < rcinput.channel_count ; i++ ){
					cout << " channel " << i << ": " << rc_array_scaled[i];
					if(i != rcinput.channel_count -1){
						cout << ",";}}
				cout << endl;
				cout << "MPU9250: ";
				cout << "Accelerometer: " << a_mpu[0] << " " << a_mpu[1] << " " << a_mpu[2];
				cout << " Gyroscope: " << g_mpu[0] << " " << g_mpu[1] << " " << g_mpu[2];
				cout << " Magnetometer: " << m_mpu[0] << " " << m_mpu[1] << " " << m_mpu[2] << endl;

				cout << "LSM9DS1: ";
				cout << "Accelerometer: " << a_lsm[0] << " " << a_lsm[1] << " " << a_lsm[2];
				cout << " Gyroscope: " << g_lsm[0] << " " << g_lsm[1] << " " << g_lsm[2];
				cout << " Magnetometer: " << m_lsm[0] << " " << m_lsm[1] << " " << m_lsm[2] << endl;

				cout << "GPS Status: " << status_gps_string << " GPS Time: " << time_gps << endl;
				cout << "Lat: ";
				cout << setprecision(9) <<  lat;
				cout << " deg\tLng: ";
				cout << setprecision(9) << lng;
				cout << " deg\tAlt(msl): " << msl_gps << " ft\tAlt(ae): " << alt_ellipsoid << endl;
				cout << "Current Waypoint: " << endl;
				cout << "Lat: " << waypoints[wind_level_index][0] << " deg\tLng: " << waypoints[wind_level_index][1] << " def\tAlt(msl): " << waypoints[wind_level_index][2] << " ft\tWind level index: " << wind_level_index << endl;
				cout << "Horz accuracy: " << horz_accuracy << " ft\t"  << " Vert Accuracy: " << vert_accuracy << " ft" << endl;

				cout << "Euler Angles (Mahony): " << "(dt = " << dt << ")" <<endl;
				cout << "MPU9250: Roll: " << roll_mpu_mahony << " Pitch: " << pitch_mpu_mahony << " Yaw: " << yaw_mpu_mahony << endl;
				cout << "LSM9DS1: Roll: " << roll_lsm_mahony << " Pitch: " << pitch_lsm_mahony << " Yaw: " << yaw_lsm_mahony << endl;
				cout << "Euler Angles (Madgwick): " << "(dt = " << dt << ")" << endl;
				cout << "MPU9250: Roll: " << roll_mpu_madgwick << " Pitch: " << pitch_mpu_madgwick << " Yaw: " << yaw_mpu_madgwick << endl;
				cout << "LSM9DS1: Roll: " << roll_lsm_madgwick << " Pitch: " << pitch_lsm_madgwick << " Yaw: " << yaw_lsm_madgwick << endl;
				cout << "Integrated Yaw (fallback):" << " MPU9250 Yaw: " << yaw_mpu_integrated_degrees << " LSM9DS1 Yaw: " << yaw_lsm_integrated_degrees << endl;

				cout << "Right Winch: " << winch_right_cmd << " Left Winch: " << winch_left_cmd << " Servo Current: " << (adc_array[5]-2500)/66 << endl;

				cout << "Control Type: " << control_type_message << endl;

				cout << "System ID, A: " << A << " B: " << B << " C: " << C;
				cout << " NDI, wn: " << wn << " zeta: " << zeta << " ki_ndi: " << ki_ndi;
				cout << " PID, kp: " << kp << " ki: " << ki << " kd: " << kd << endl;
				cout << "Heading type: " << heading_type_message << endl;
				cout << "Yaw Desired: " << yaw_desired << " Yaw Error: " << yaw_error << " Yaw Error Rate: " << yaw_error_rate << " Yaw Error Sum: " << yaw_error_sum << endl;
				cout << "Proportional: " <<  kp*yaw_error << " integral: " << ki*yaw_error_sum << " derivative: " << kd*yaw_error_rate << endl;
				cout << "Number of wraps: " << num_wraps;
				cout << " Step counter: " << step_counter << endl;
				multisine_counter++;}

			cout << tse << " - " << time_start << " = " << time_now << endl;

//dbmsg_local = true;
			if(dbmsg_global || dbmsg_local){
				// Alternate debug message, just print out the most current duration for each loop occasionally
				cout << "Printing the most current expected vs. actual time since last execution for each loop" << endl;
				for(int i = 0 ; i < NUM_LOOPS ; i++){
					cout << "For " << frequency[i] << "(Hz) loop, expected: " << duration[i] << "(us), actual: " << watcher[i] << endl;}}
//dbmsg_local = false;

			watcher[8] = time_now - timer[8]; // used to check loop frequency
			timer[8] = time_now;
		}

	}

	// tidy up and close the file when we exit the inner while loop
	fout.close();
	}
	return 0;
}
