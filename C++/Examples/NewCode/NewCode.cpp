// custom function includes
#include "Navio/wrapped_variable.h"
#include "Navio/data_file.h"
#include "Navio/herrington_utils.h"
#include "Navio/testing.h"
#include "Navio/vehicle.h"
#include "Navio/digital_filter.h"

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
#include <cmath>
// RC Input includes
//#include <Navio/RCInput.h>
// IMU Includes
#include "Navio/MPU9250.h"
#include "Navio/LSM9DS1.h"
#include "Navio/Util.h"
// AHRS Includes
#include "AHRS2.hpp"
// ADC Includes
#include <Navio/ADC.h>
// GPS Includes
#include "Navio/Ublox.h"
// Serial Includes
#include <wiringSerial.h>
#include <wiringPi.h>

#define GPSKEY 1
#define MSG_BAR "======================================="

using namespace std;

float dt_control = 0;

int m_order = 2;
char m_type = 'l';
float m_fc  = 2; //hz
float m_fs  = 100; //hz

digital_filter reference_model(m_order, m_type, m_fc, m_fs);

//const float l1 = .003;
//const float l2 = .003;
const float l1 = .00015;
const float l2 = .00015;
const float kp_outer_loop = 1;
const float ki_outer_loop = 0;

float p1 = .017;
float p2 = 0;
float p1_d = 0;
float p2_d = 0;

float error_outer_loop = 0;
float error_sum_outer_loop = 0;
float yaw_rate_desired = 0;
float yaw_rate_model = 0;
float cmd_adapt = 0;
float error_model = 0;

//---------------------------------------------------------------------------------------Step Generator (Eloy version) Parameters
float out = 0;

const int step = 1;
const int plus_minus = 0;
const int multi_step = 1;
const float des_period = 10;
const float des_amp = .35;
const int num_steps_mg = 3;
const float des_delay = 5;

// log file location (relative path)
std::string file_location = "./LogFiles/";

data_file mpu_offsets("./mpu_mag_cal.csv",4,3);
data_file lsm_offsets("./lsm_mag_cal.csv",4,3);
data_file waypoints_df("./waypoints.csv",1000,3);

// create an object of the type
wrapped_variable continuous_yaw(360,160);

//---------------------------------------------------------------------------------------------------User Configurable Parameters
const bool dbmsg_global = false; // set flag to display all debug messages
bool dbmsg_local  = false; // change in the code at a specific location to view local messages only
char control_type = 'p'; // valid options:  p=PID , n=NDI , g=glide (no spin), m=multisine , s=input sweep, c=rc control
char heading_type = 'u'; // valid otpoins 1=N, 2=E, 3=S, 4=W, u=user, c=rc control, n=navigation algorithm, d=dumb navigation
float user_heading = 115; //degress, only used if us is the heading type
bool live_gains = false;
string filename_str = "";

//----------------------------------------------------------------------------------------------------------------------New Stuff
const float final_heading = 0; // must be set manually for into the wind
//float yaw_l_desired; // will be rate limited for maximum turning rate circular paths
const float max_degrees_per_sample = 1; // corresponds to 100 degrees per second;
const float radius_of_max_turn = .00001; // corresponds to about a 50ft. turn radius
float yaw_l_desired_prev = 0;

//-------------------------------------------------------------------------------------------------------Common Control Variables
float yaw_desired    = 0;
string heading_type_message = "null";
float yaw_error      = 0;
float yaw_error_previous = 0;
float yaw_error_sum  = 0;
float yaw_error_rate = 0;
int num_wraps = 0;
float yaw_prev = 0;

float yaw_l_desired    = 0;
float yaw_l_error      = 0;
float yaw_l_error_previous = 0;
float yaw_l_error_sum  = 0;
float yaw_l_error_rate = 0;
float yaw_l_prev = 0;
// --------------------------------------------------------------------------------------------------Multi Step input declarations
float multi_time = 0;
float N = 0;
float num_steps = 3;
float max_amp = 0.35;
float step_size = max_amp/num_steps;
float state = 0;
float period = 20; // seconds, on/off duraton, must be a minimum of 3
float b1 = 0;
float pulse_train = 0; // current value for 1 second resolution pulse train signal constructed from noisy microsecond time
float prev_pulse_train = 0; // previous value of pulse_train
unsigned long long multi_time_offset = 0;
// ---------------------------------------------------------------------------------------------Frequency Sweep Input Declarations
float time_sweep = 0; // synthetic time-like signal created by incrementing by the chosen discrete step
const float total_time = 30; // total duration of the designed sweep in seconds
const float sweep_amp = 0.35; // maximum positive amplitude of the sweep, signal is symmetrical as expected
float phi = 0;
float wmin = 0.6; // Hz
float wmax = 60; // Hz
float c1 = 4;
float c2 = 0.0187;

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

// Roberts-July 75% gains on X50
//double kp_start = .07937*0.75;
//double ki_start = .05591*0.75;
//double kd_start = .02814*0.75;

// Control Validation Program - Run 2, OK Gains
//double kp_start = .2686;
//double ki_start = .1143;
//double kd_start = .1086;

// Control Validation Program - Run 3, Vigorous Gains
//double kp_start = .314;
//double ki_start = .114;
//double kd_start = .114;

string control_type_message = "null";

//---------------------------------------------------------------------------------------------------------Barometer Declarations
// most barometer variables moved to barometer class
float msl = 0.0; // mean sea level altitue (ft) [should be close to 920ft for UMKC Flarsheim]

// rc input declaration moved to remote_control class

//---------------------------------------------------------------------------------------------------------------IMU Declarations
//#define DECLINATION -12.71 //magnetic declination for camp roberts
//#define DECLINATION -10.33 //magnetic declination for Yuma Proving Ground
#define DECLINATION -10.09 //magnetic declination for Eloy AZ
//#define DECLINATION 1.70 //magnetic declination for KC
#define WRAP_THRESHOLD 160.00 // wrap threshold for wrap counter (yaw is +/-180, need to make it continuous
// vars to hold mpu values
float a_mpu[3] , a_mpu_ahrs[3];
float g_mpu[3] , g_mpu_ahrs[3];
float m_mpu[3] ;
// vars to hold lsm values
float a_lsm[3] , a_lsm_ahrs[3];
float g_lsm[3] , g_lsm_ahrs[3];
float m_lsm[3] ;

//also need a simple que to store old data points
float gyro_z_lsm_old[3] = {0,0,0};
float gyro_z_mpu_old[3] = {0,0,0};

//-------------------------------------------------------------------------------------------------------------Servo Declarations
#define WINCH_RIGHT 1 // right hand winch servo, 1 is the servo rail position
#define WINCH_LEFT 0 // left hand winch servo, 0 is the servo rail position
#define MAX_DEFLECTION 0.2 // this is used for rigging
#define LINE_NEUTRAL 1.500 // center point for the actuator, range is 1.000-2.000
#define LINE_OFFSET .055 // think this was used when glide was applied by the "static" actuator after initialization
#define DEFLECTION_LIMIT .15 // correction, this is used for saturation
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

//---------------------------------------------------------------------------------------------------------------GPS Declarations double
static volatile double time_gps = 0;
static volatile double lat = 0;
static volatile double lng = 0;
static volatile double alt_ellipsoid = 0;
static volatile double msl_gps = 10000;
static volatile double horz_accuracy = 0;
static volatile double vert_accuracy = 0;
static volatile int status_gps = 0x00; // default condition - no fix
string status_gps_string = "no fix";

//----------------------------------------------------------------------------------------------------------Waypoint Declarations
double waypoints[50][3]; // waypoint array is 50x3
//double target[2] = {35.7178528,-120.76411}; // from step input payload drop
//double target[2] = {35.7185462, -120.763162} //simulated fixed heading
//double target[2] = {35.7185462, -120.763599}; // dumb nav, same as drop point
//double target[2] = {35.6414, -120.68810};
//double target[2] = {33.397694,-114.273444};
double target[2] = {32.791300, -111.434883}; // Eloy Area 51 IP
//double target[2] = {39.016998,-94.585846}; // intersection of 61st street and Morningside
//double target[2] = {32.791300, -111.434883}; // Eloy Area 51 IP
//double target[2] = {39.016998,-94.585846}; // intersection of 61st street and Morningside
//double target[2] = {35.7163752, -120.7635108}; // CR in front of vehicle hangar

//-----------------------------------------------------------------------------------------------------------Logfile Declarations

PI_THREAD (decodeGPS)
{
	(void)piHiPri(99);
	static vector<double> pos_data; // this vector will contain undecoded gps information
	static Ublox gps;
	cout << "Initializing GPS......................." << endl;
	if(!gps.testConnection()){ // check if the gps is working
		cout << "    --ERROR, GPS not initialized--    " << endl;
		if(heading_type == 'n'){
			// stop program executiong if gps won't initialize and navigation heading type is selected
			cout << "Fatal exception, navigation impossible" << endl;
			cout << "without GPS, try restarting..........." << endl;
			// return EXIT_FAILURE;}}
	}}
	else{ // gps is good!
		cout << "  --GPS successfully initialized--    " << endl;}
	while(true)
	{
		if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
		{
			piLock(GPSKEY);
			time_gps = pos_data[0]/1000.00000;
			lng = pos_data[1]/10000000.00000;
			lat = pos_data[2]/10000000.00000;
			alt_ellipsoid = (pos_data[3]/1000.00000)*3.28;
			msl_gps = (pos_data[4]/1000.00000)*3.28;
			horz_accuracy = (pos_data[5]/1000.00000)*3.28;
			vert_accuracy = (pos_data[6]/1000.00000)*3.28;
			piUnlock(GPSKEY);
		}
		if (gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1)
		{
			piLock(GPSKEY);
 			status_gps = (int)pos_data[0];
			piUnlock(GPSKEY);
		}

	}
}

int main( int argc , char *argv[])
{
	// when they were initialized, the data_file objects mpu_offsets and lsm_offsets automatically read in the file contents
	// now to make the information contained in the calibration file useful we need to split it into a magnetometer offset
	// vector and a 3x3 rotation matrix
	mpu_offsets.row2array(mag_offset_mpu,1);
	lsm_offsets.row2array(mag_offset_lsm,1);
	for( int i = 0; i < 3 ; i++ )
	{
		mpu_offsets.row2array(mag_rotation_mpu[i],i+2);
		lsm_offsets.row2array(mag_rotation_lsm[i],i+2);
	}

	double kp = kp_start*.0175; // convert all 3 to radians
	double ki = ki_start*.0175;
	double kd = kd_start*.0175;
	// float ki_ndi = ki_ndi_start*.0175; // convert to radians
	int multisine_counter = 0;

	//--------------------------------------------------------------------------------------------------Read and Handle Program Options
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



	cout << endl << MSG_BAR << endl;
	cout << "         Begin Initialiation           ";
	cout << endl << MSG_BAR << endl;

	// create an object of type vehicle called parachute
	vehicle parachute;
/*
	parachute.pwm_out.init(0);
	parachute.pwm_out.init(1);

	parachute.pwm_out.enable(0);
	parachute.pwm_out.enable(1);

	parachute.pwm_out.set_period(0,50);
	parachute.pwm_out.set_period(1,50);
*/

	// loop scheduler variables have been moved to scheduler class
	// barometer initialization moved to barometer class

	parachute.timer.update_time();

	// rc initialization has been moved to remote_control class

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

	// servo initialization moved to vehicle class

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
		offset_mpu[0] -= g_mpu[0];
		offset_mpu[1] -= g_mpu[1];
		offset_mpu[2] -= g_mpu[2];
		offset_lsm[0] -= g_lsm[0];
		offset_lsm[1] -= g_lsm[1];
		offset_lsm[2] -= g_lsm[2];

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

	// NOTE:  GPS initialization moved to threaded function to avoid scoping issues

	int wind_level_index = 0;

	piThreadCreate(decodeGPS);

	//------------------------------------------------------------------------------------------------------------Serial Initialization
	cout << "Initializing Serial Output............" << endl;
	int serialHandle = serialOpen("/dev/ttyAMA0", 9600);
	cout << "--Serial Output successfully opened --" << endl;

	//------------------------------------------------------------------------------------------------------------------Welcome Message
	usleep(500000);
	cout << endl;
	cout << MSG_BAR << endl;
	cout << "Initialization completed..............." << endl;
	cout << MSG_BAR << endl << endl;
	usleep(500000);
	cout << MSG_BAR << endl;
	cout << "Main Loop starting now................." << endl;
	cout << MSG_BAR << endl << endl;
	usleep(500000);

	//----------------------------------------------------------------------------------------------------------Log File Initialization
	while(true)
	{
		int standby_message_timer = 0; // used to limit the frequency of the standby message
		while(!(adc_array[4] < 4000)){
//		while(!((parachute.rc.get_raw(5)>1500)&&(adc_array[4]<4000))){
//bool temp_flag = false; // uncomment for testing with no transmitter
//while(!temp_flag){ // uncomment for testing with no transmitter
			if(standby_message_timer > 250){
				cout << endl << "---------------------------------------" << endl << "           Autopilot Inactive         " << endl;
				cout << "  Dynamic Lines at Neutral Deflection " << endl;
				cout << "       Battery Voltage:  " << adc_array[2]/100 << " V" << endl;
				cout << "         Waiting for Killswitch       " << endl << "--------------------------------------" << endl;
				standby_message_timer = 0;}
			// when the autopilot is inactive, set both winches to the neutral deflection
//			cout << "\t" << LINE_NEUTRAL << endl;
//			parachute.pwm_out.set_period(0,50);
//			parachute.pwm_out.set_period(0,50);
//			parachute.pwm_out.set_duty_cycle(0,1.5);
//			parachute.pwm_out.set_duty_cycle(1,1.5);
			parachute.pwm_out.set_duty_cycle(WINCH_RIGHT, LINE_NEUTRAL);
			parachute.pwm_out.set_duty_cycle(WINCH_LEFT, LINE_NEUTRAL);

			// since this loop executes based on an rc and an adc condition, we have to poll these devices for new status
			// rc_array[5] = parachute.rc.get_raw(5);

			parachute.rc.update();

			adc_array[4] = adc.read(4);
			adc_array[2] = adc.read(2);
			// step counter needs to be set to zero also, this is a hack because the numbering is messed up in the code
			// we start at negative one here and presumably the counter is incremented before 1st execution
			usleep(5000);
			standby_message_timer++; // increment the message delay timer
			// everything that needs to be set to zero by the killswitch goes here
			continuous_yaw.reset_wrap_counter();
			time_sweep = 0;
			multisine_counter = 0; // time counter for the multisine input

			error_outer_loop = 0;
			error_sum_outer_loop = 0;
			yaw_rate_desired = 0;
			error_model = 0;
			cmd_adapt = 0;
			p1 = .017;
			p2 = 0;
			p1_d = 0;
			p2_d = 0;


//			wind_level_index = 0; // wind level for navigation, can only be incremented by the main loop to avoid "waypoint indecision"
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
		char *today = asctime(localtime(&result));
		cout << filename_str << endl;
		filename_str = generate_filename(logfile_prefix_string);

		// create the file output stream object
		ofstream fout;
		// open the file
		cout << "Log file created at: " << endl << filename_str << endl << endl;
		fout.open(filename_str,ios::out);
		// header string
		fout <<
			"today,microseconds_since_start,"
			"msl,"
			"rc0,rc1,rc2,rc3,rc4,rc5,"
			"rc0_scaled,rc1_scaled,rc2_scaled,rc3_scaled,rc4_scaled,rc5_scaled,"
			"adc_array0,adc_array1,adc_array2,adc_array3,adc_array4,adc_array5,"
			"roll_mpu_mahony,pitch_mpu_mahony,yaw_mpu_mahony,"
			"roll_lsm_mahony,pitch_lsm_mahony,yaw_lsm_mahony,"
			"roll_mpu_madgwick,pitch_mpu_madgwick,yaw_mpu_madgwick,"
			"roll_lsm_madgwick,pitch_lsm_madgwick,yaw_lsm_madgwick,"
			"a_mpu[0],a_mpu[1],a_mpu[2],a_lsm[0],a_lsm[1],a_lsm[2],"
			"g_mpu[0],g_mpu[1],g_mpu[2],g_lsm[0],g_lsm[1],g_lsm[2],"
			"m_mpu[0],m_mpu[1],m_mpu[2],m_lsm[0],m_lsm[1],m_lsm[2],"
			"winch_right_cmd,winch_left_cmd,"
			"kp,ki,kd,"
			"yaw_desired,yaw_error,yaw_error_previous,yaw_error_rate,yaw_error_sum,"
			"control_type,heading_type,"
			"time_gps,lat,lng,alt_ellipsoid,msl_gps,horz_accuracy,vert_accuracy,status_gps,status_gps_string,"
			"lat_waypoint,lng_waypoint,"
			"yaw_l_desired,yaw_l_error,yaw_l_error_previous,yaw_l_error_rate,yaw_l_error_sum,"
			"l1,l2,kp_outer_loop,ki_outer_loop,error_outer_loop,error_sum_outer_loop,yaw_rate_desired,yaw_rate_model,cmd_adapt,error_model,"
			"p1,p2,p1_d,p2_d"
			<< endl;
		usleep(20000);
		//everything that needs to be set to zero by the killswitch goes here
		continuous_yaw.reset_wrap_counter();
		time_sweep = 0;
		multisine_counter = 0;

		error_outer_loop = 0;
		error_sum_outer_loop = 0;
		yaw_rate_desired = 0;
		cmd_adapt = 0;
		error_model = 0;
		p1 = .017;
		p2 = 0;
		p1_d = 0;
		p2_d = 0;


		for(int i = 0 ; i < 3 ; i++){
			gyro_z_lsm_old[i] = 0;
			gyro_z_mpu_old[i] = 0;
		}
		yaw_error_sum       = 0; //prevent integral wind up
		yaw_error           = 0;
		yaw_error_previous  = 0;
		num_wraps = 0;
		yaw_prev = 0;

	while(adc_array[4] < 4000) // while condition for arm lanyard only
//	while((parachute.rc.get_raw(5)>1500)&&(adc_array[4]<4000)) // this is to be used when an RC transmitter is present as well as the arm lanyard
//while(true)
	{
/* 		// refresh time now to prepare for another loop execution
		gettimeofday(&time_obj, NULL); // must first update the time_obj
		tse        = time_obj.tv_sec*1000000LL + time_obj.tv_usec; // update tse (us)
		//tse        = tse + 2000000000; // uncomment to test integer overflow fix
		time_now   = tse - time_start; // calculate the time since execution start by subtracting off tse */

		parachute.timer.update_time();

		if( (parachute.timer.get_current_time()-parachute.timer.get_timer(0)) > parachute.timer.get_duration(0))
		{
			//----------------------------------------------------------------------------------------------------------------AHRS Update
			dt = parachute.timer.get_current_time()-parachute.timer.get_timer(0);
			dt = dt/1000000.0; // convert from useconds

			if(wind_level_index<49){
				if(msl_gps < waypoints[wind_level_index][2]){
					wind_level_index++;}}
			if(wind_level_index>1){
				if(msl_gps > waypoints[wind_level_index][2]){
					wind_level_index--;}}

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

			// update the que holding the old gyro values with the new gyro information
			gyro_z_mpu_old[2] = gyro_z_mpu_old[1];
			gyro_z_mpu_old[1] = gyro_z_mpu_old[0];
			gyro_z_mpu_old[0] = g_mpu_ahrs[2]-offset_mpu[2];
			gyro_z_lsm_old[2] = gyro_z_lsm_old[1];
			gyro_z_lsm_old[1] = gyro_z_lsm_old[0];
			gyro_z_lsm_old[0] = g_lsm_ahrs[2]-offset_lsm[2];

			parachute.timer.update_watcher(0); // used to check loop frequency
			parachute.timer.update_timer(0);
		}

		if( (parachute.timer.get_current_time()-parachute.timer.get_timer(1)) > parachute.timer.get_duration(1))
		{
			// GPS which was previously in this section is now threaded
			parachute.timer.update_watcher(1); // used to check loop frequency
			parachute.timer.update_timer(1);
			}

		if( (parachute.timer.get_current_time()-parachute.timer.get_timer(2)) > parachute.timer.get_duration(2))
		{

			// testing maneuver generator at Natick AGU speed of 5Hz
			out = maneuver_generator(step,plus_minus,multi_step,des_period,des_amp,num_steps_mg,des_delay);

//			if(msl_gps < 2700)
//			{
//				heading_type = 'd'; // dumb nav
//				control_type = 'p'; // PID
//			}
//			else
//			{
//				heading_type = 'x'; // default north
//				control_type = 's'; // sweep (set for mstep)
//			}

			parachute.timer.update_watcher(2); // used to check loop frequency
			parachute.timer.update_timer(2);
		}

		if( (parachute.timer.get_current_time()-parachute.timer.get_timer(3)) > parachute.timer.get_duration(3))
		{

			piLock(GPSKEY);

			// decode GPS status at 100 Hz, no need to do this in the thread, also causes scope problems
			switch(status_gps){
				case 0x00:
					status_gps_string = "no fix";
					break;
				case 0x01:
					status_gps_string = "dead reckoning only";
					break;
				case 0x02:
					status_gps_string = "2D-fix";
					break;
				case 0x03:
					status_gps_string = "3D-fix";
					break;
				case 0x04:
					status_gps_string = "GPS + dead reckoning combined";
					break;
				case 0x05:
					status_gps_string = "Time only fix";
					break;
				default:
					status_gps_string = "Reserved value. Current state unknown";
					break;
			}

			piUnlock(GPSKEY);

			// barometer read placed inside barometer class
			parachute.baro.update_sensor();

			// all rc reading is accomplished by member function from remote_control class
			parachute.rc.update();

			//-----------------------------------------------------------------------------------------------------------------ADC Update
			for(int i = 0 ; i < ARRAY_SIZE(adc_array) ; i++){
				adc_array[i] = adc.read(i);}

			//------------------------------------------------------------------------------------------------AHRS Euler Angle Conversion
			// convert the quaternion position to euler angles so it makes sense to humans
			ahrs_mpu_mahony.getEuler(&roll_mpu_mahony,&pitch_mpu_mahony,&yaw_mpu_mahony);
			ahrs_lsm_mahony.getEuler(&roll_lsm_mahony,&pitch_lsm_mahony,&yaw_lsm_mahony);
			ahrs_mpu_madgwick.getEuler(&roll_mpu_madgwick,&pitch_mpu_madgwick,&yaw_mpu_madgwick);
			ahrs_lsm_madgwick.getEuler(&roll_lsm_madgwick,&pitch_lsm_madgwick,&yaw_lsm_madgwick);

			//----------------------------------------------------------------------------------------------------------------Controllers
//			float dt_control = parachute.timer.get_current_time()-parachute.timer.get_timer(3); // dt for this loop
			dt_control = parachute.timer.get_current_time()-parachute.timer.get_timer(3);
			dt_control = dt_control/1000000.0; // convert from useconds
			if(abs(dt_control) > 100)
			{
				dt_control = 0;
			}
//			cout << endl << dt_control << endl;

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
					yaw_desired = parachute.rc.get_scaled(3); // yaw stick
					heading_type_message = "RC " + to_string(parachute.rc.get_scaled(3));
					yaw_desired = parachute.rc.get_scaled(3);
					break;
				case 's': // Tuesday 8/28 Payload3 heading control steps %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					piLock(GPSKEY);
					heading_type_message = "Step Input (altitude driven)";
//					if(msl_gps > 1400){
//						yaw_desired = user_heading;}
//					if(msl_gps <= 1400){
//						yaw_desired = user_heading + 90;}

					if(msl_gps > 1800){
						yaw_desired = user_heading;}
					if(msl_gps <= 1800){ // && msl_gps > 1400){
						yaw_desired = user_heading + 90;}
			//		if(msl_gps <= 1400&& msl_gps > 1000){
		//				yaw_desired = user_heading + 180;}
	//				if(msl_gps <= 1000 ){
//						yaw_desired = user_heading - 90;}
//					if(rc_array[2] > 1600){
//						yaw_desired = 90;
//					} else{
//						yaw_desired = 0; // set yaw desired here

					piUnlock(GPSKEY);
					break;
				case 'n':
					piLock(GPSKEY);
					heading_type_message = "Navigation Algorithm";
					//write code to determine heading here
					//lat_target
					//lon_target
					if(waypoints[wind_level_index][0] == 0)
					{
						yaw_desired = 100;
					}else{
						yaw_desired = atan2(waypoints[wind_level_index][1]-lng,waypoints[wind_level_index][0]-lat);
						yaw_desired = (yaw_desired/.0175);
					}
					piUnlock(GPSKEY);
					break;
				case 'd':
					piLock(GPSKEY);
					heading_type_message = "Dumb Navigation";
					yaw_desired = atan2(target[1]-lng,target[0]-lat);
					yaw_desired = (yaw_desired/.0175);
					piUnlock(GPSKEY);
					break;
				case 'p':
					piLock(GPSKEY);
					heading_type_message = "Dubin's Path Generation";
					if( abs(yaw_mpu_madgwick - final_heading) > 240) // connect cw-ccw circles
					{
						yaw_l_desired = atan2(waypoints[wind_level_index][1]-lng,waypoints[wind_level_index][0]-lat);
						yaw_l_desired = (yaw_l_desired/.0175);
//						cout << " cw-ccw ";
					}
					else if( ((yaw_mpu_madgwick - final_heading) >= -240) && ((yaw_mpu_madgwick - final_heading) <= 0) ) // connect cw dcircles
					{
						yaw_l_desired = atan2(waypoints[wind_level_index][1]-lng,waypoints[wind_level_index][0]-lat);
						yaw_l_desired = yaw_l_desired - asin((2*radius_of_max_turn)/sqrt(pow((waypoints[wind_level_index][1] - lng),2) + pow((waypoints[wind_level_index][0] - lat),2)));
						yaw_l_desired = (yaw_l_desired/.0175);
//						cout << " cw ";
					}
					else if( ((yaw_mpu_madgwick - final_heading) > 0) && ((yaw_mpu_madgwick - final_heading) <= 240) ) // connect ccw circles
					{
						yaw_l_desired = atan2(waypoints[wind_level_index][1]-lng,waypoints[wind_level_index][0]-lat);

						float arg = pow(lng-waypoints[wind_level_index][1],2)+pow(lat-waypoints[wind_level_index][0],2);
//						cout << "1 - " << arg << endl;
						arg = sqrt(arg);
//						cout << "2 - " << arg << endl;
						arg = (2*radius_of_max_turn)/arg;
//						cout << "3 - " << arg << endl;
						arg = asin(arg);
//						cout << "4 - " << arg << endl;

//						yaw_l_desired = yaw_l_desired + asin((2*radius_of_max_turn)/sqrt(pow((lng - waypoints[wind_level_index][1]),2) + pow((lat - waypoints[wind_level_index][0]),2)));

						yaw_l_desired = yaw_l_desired + arg;

						yaw_l_desired = (yaw_l_desired/.0175);
//						cout << " ccw ";
					}
					if( msl_gps < 1300 )
					{
//						yaw_l_desired = final_heading;
					}
					piUnlock(GPSKEY);
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
			yaw_mpu_mahony = yaw_mpu_mahony - DECLINATION;
			yaw_lsm_mahony = yaw_lsm_mahony - DECLINATION;
			yaw_mpu_madgwick = yaw_mpu_madgwick - DECLINATION;
			yaw_lsm_madgwick = yaw_lsm_madgwick - DECLINATION;

			continuous_yaw.process_new_input(yaw_mpu_madgwick);

			if((yaw_l_desired_prev - yaw_l_desired) > max_degrees_per_sample)
			{
				yaw_l_desired = yaw_l_desired - max_degrees_per_sample;
			}
			else if((yaw_l_desired_prev - yaw_l_desired) < -max_degrees_per_sample)
			{
				yaw_l_desired = yaw_l_desired + max_degrees_per_sample;
			}

			yaw_l_desired_prev = yaw_l_desired;

			// calculate yaw error for controller
			yaw_prev              = yaw_mpu_madgwick;
			yaw_error_previous    = yaw_error;
			yaw_error             = yaw_desired -(yaw_mpu_madgwick+(360*num_wraps));
			yaw_error_rate        = (yaw_error - yaw_error_previous)/dt_control;
			yaw_error_sum         = (((yaw_error + yaw_error_previous)*dt_control)/2) + yaw_error_sum;

			// yaw error for rate limited control
			yaw_l_prev	     	 = yaw_mpu_madgwick;
			yaw_l_error_previous = yaw_l_error;
			yaw_l_error	     	 = yaw_l_desired - (yaw_mpu_madgwick+(360*num_wraps));
			yaw_l_error_rate     = (yaw_l_error - yaw_l_error_previous)/dt_control;
			yaw_l_error_sum	     = (((yaw_l_error + yaw_l_error_previous)*dt_control)/2) + yaw_l_error_sum;

			// saturate the yaw error sum as a reasonable value
			if(yaw_error_sum > 150){
				yaw_error_sum = 150;
			} if(yaw_error_sum < -150){
				yaw_error_sum = -150;
			}

			// saturation block for rate limited yaw
			if(yaw_l_error_sum > 150){
				yaw_error_sum = 150;
			} if(yaw_l_error_sum < -150){
				yaw_l_error_sum = -150;
			}

			// hack to avoid changing the variable name "t" in the multisine
			float t = parachute.timer.get_current_time()*1e-6;

			// gains are modified live according to rc transmitter stick positions
			if(live_gains)
			{
				kp    =  (kp_start+parachute.rc.get_scaled(2))*(.0175);
				ki    =  (ki_start+parachute.rc.get_scaled(3))*(.0175);
				kd    =  (kd_start+parachute.rc.get_scaled(0))*(.0175);
				// wn    =  wn_start+rc_array_scaled[2];
				// zeta  =  zeta_start+rc_array_scaled[1];
				// ki_ndi = ki_ndi_start+rc_array_scaled[3];
			}
			else
			{
				kp = kp_start*.0175;
				ki = ki_start*.0175;
				kd = kd_start*.0175;
				// wn = wn_start;
				// zeta = zeta_start;
				// ki_ndi = ki_ndi_start;
			}
			// control type is determined by configuration parameter from file (or default in code)
			switch(control_type){
				case 'a':
					control_type_message = "Adaptive (MRAC) Rate Control";

//					cout << endl << dt_control << endl;

					// outer loop error
					error_outer_loop = yaw_desired - continuous_yaw.get_current_continuous();
					error_sum_outer_loop = error_sum_outer_loop + error_outer_loop * dt_control;

					// outer loop controller
					yaw_rate_desired = kp_outer_loop * error_outer_loop + ki_outer_loop * error_sum_outer_loop;

					// pass desired yaw through the model
					yaw_rate_model = reference_model.filter_new_input(yaw_rate_desired);

					//MRAC control law
					cmd_adapt = p1*yaw_rate_desired + p2;

					// error for adaptation
					error_model = g_mpu[2]*57.3 - yaw_rate_model;

					// derivative of gains
					p1_d = -l1*yaw_rate_desired*error_model;
					p2_d = -l2*error_model;

					// adapt the gains
					p1 = p1 + p1_d * dt_control;
					p2 = p2 + p2_d * dt_control;

//					cout << endl;
//					cout << "p1 = " << p1 << "\tp2 = " << p2 << endl;
//					cout << "p1_d*dt_control = " << p1_d*dt_control << "\tp2_d*dt_control = " << p2_d*dt_control << endl;

					cmd_adapt = cmd_adapt/100000;

					winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET + cmd_adapt;
//					winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET;
					winch_right_cmd = LINE_NEUTRAL + LINE_OFFSET;
//					winch_right_cmd = LINE_NEUTRAL + cmd_adapt;

					break;
				case 'f': // Frequency Sweep successfully bench tested 9/21/18
					control_type_message = "Frequency Sweep";
					 phi = wmin*time_sweep + c2*(wmax - wmin)*((total_time/c1)*exp(c1*time_sweep/total_time) - time_sweep);
					if (time_sweep <= total_time){
					time_sweep = time_sweep + 0.01;}
					else {
						winch_left_cmd = 0;
						winch_right_cmd = 0;}

					winch_left_cmd  = LINE_NEUTRAL + sweep_amp*sin(phi);
					winch_right_cmd = LINE_NEUTRAL + sweep_amp*sin(phi);
					break;

				//----------------------------------------------------------
				case 'j': // *********** Multi Step ****** successfully bench tested 9/21/18
					control_type_message = "Multistep";
							multi_time = round((parachute.timer.get_current_time() - multi_time_offset)/1e6); // make it start at exactly zero
							prev_pulse_train = pulse_train;
							pulse_train = ((fmod(multi_time , (period/2))) == 0);
							b1 = (pulse_train == 1 && prev_pulse_train == 0);
							if (b1 == 1){

								if (state == 0){
									state = 1;
								}
								else {
									state = 0;
									N = N+1;
									if (N > num_steps) {
									  N = -num_steps;}
									}}

						winch_left_cmd  = LINE_NEUTRAL + step_size*N*state;
						winch_right_cmd = LINE_NEUTRAL + step_size*N*state;
						break;
				case 'p':
					control_type_message = "PID";
					if(0){
						winch_right_cmd = LINE_NEUTRAL + (kp*yaw_error + ki*yaw_error_sum + kd*yaw_error_rate);
						winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET;
					} else {
						winch_left_cmd = LINE_NEUTRAL + (kp*yaw_error + ki*yaw_error_sum + kd*yaw_error_rate);
						winch_right_cmd = LINE_NEUTRAL + LINE_OFFSET;}
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
					// now using maneuver_generator for this mode
					winch_right_cmd = LINE_NEUTRAL + out;
					winch_left_cmd = LINE_NEUTRAL + out;
					break;
				case 'c': // rc control
					control_type_message = "RC control";
					//for RC control, throttle is left servo, elevator is right servo, down on the sticks is line pull
					winch_right_cmd = LINE_NEUTRAL + parachute.rc.get_scaled(2);
					//winch_left_cmd = LINE_NEUTRAL - parachute.rc.get_scaled(2);
					winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET;
					break;
				case 'm': // multisine
					control_type_message = "Multisine";
					// multisine or step input, only enable when switch D is in the 1 or 2 position
					if(parachute.rc.get_raw(4) > 1500) // switch A 0 for left, 1 for right
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
				case 'l': // rate limited (maximum turn rate dynamics)
					control_type_message = "Rate Limited";
					if(0){
						winch_right_cmd = LINE_NEUTRAL + (kp*yaw_l_error + ki*yaw_l_error_sum + kd*yaw_l_error_rate);
						winch_left_cmd = LINE_NEUTRAL + LINE_OFFSET;
					} else {
						winch_left_cmd = LINE_NEUTRAL + (kp*yaw_l_error + ki*yaw_l_error_sum + kd*yaw_l_error_rate);
						winch_right_cmd = LINE_NEUTRAL - LINE_OFFSET;}
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
			parachute.pwm_out.set_duty_cycle(WINCH_RIGHT, winch_right_cmd);
			parachute.pwm_out.set_duty_cycle(WINCH_LEFT, winch_left_cmd);

			//-------------------------------------------------------------------------------------------------------Data Log File Output
			fout << today << "," << parachute.timer.get_current_time() << "," << msl << ",";
			for(int i = 0 ; i < NUM_CHANNELS ; i++){
				fout << parachute.rc.get_raw(i) << ",";}
			for(int i = 0 ; i < NUM_CHANNELS ; i++){
				fout << parachute.rc.get_scaled(i) << ",";}
			for(int i = 0 ; i < ARRAY_SIZE(adc_array) ; i++){
				fout << adc_array[i] << ",";}
			fout << roll_mpu_mahony << "," << pitch_mpu_mahony << "," << yaw_mpu_mahony << ",";
			fout << roll_lsm_mahony << "," << pitch_lsm_mahony << "," << yaw_lsm_mahony << ",";
			fout << roll_mpu_madgwick << "," << pitch_mpu_madgwick << "," << yaw_mpu_madgwick << ",";
			fout << roll_lsm_madgwick << "," << pitch_lsm_madgwick << "," << yaw_lsm_madgwick << ",";
			fout << a_mpu[0] << "," << a_mpu[1] << "," << a_mpu[2] << "," << a_lsm[0] << "," << a_lsm[1] << ",";
			fout << a_lsm[2] << "," << g_mpu[0] << "," << g_mpu[1] << "," << g_mpu[2] << "," << g_lsm[0] << "," << g_lsm[1] << ",";
			fout << g_lsm[2] << "," << m_mpu[0] << "," << m_mpu[1] << "," << m_mpu[2] << "," << m_lsm[0] << "," << m_lsm[1] << "," << m_lsm[2] << ",";
			fout << winch_right_cmd << "," << winch_left_cmd << ","  ;
			fout << kp << "," << ki << "," << kd << ",";
			fout << yaw_desired << "," <<  yaw_error << "," << yaw_error_previous << "," << yaw_error_rate << "," << yaw_error_sum << ",";
			fout << control_type << "," << heading_type << ",";

			piLock(GPSKEY);
			fout << setprecision(9) << time_gps << "," << lat << "," << lng << "," << alt_ellipsoid << "," << msl_gps << "," << horz_accuracy << "," << vert_accuracy << "," << status_gps << "," << status_gps_string << ",";
			piUnlock(GPSKEY);

			fout << waypoints[wind_level_index][0] << "," << waypoints[wind_level_index][1] << ",";
			fout << yaw_l_desired << "," <<  yaw_l_error << "," << yaw_l_error_previous << "," << yaw_l_error_rate << "," << yaw_l_error_sum << ",";

			fout << l1 << "," << l2 << "," << kp_outer_loop << "," << ki_outer_loop << "," << error_outer_loop << ",";
			fout << error_sum_outer_loop << "," << yaw_rate_desired << "," << yaw_rate_model << "," << cmd_adapt << ",";
			fout << error_model << "," << p1 << "," << p2 << "," << p1_d << "," << p2_d;

			fout << endl;



			parachute.timer.update_watcher(3); // used to check loop frequency
			parachute.timer.update_timer(3);
		}

		if( (parachute.timer.get_current_time()-parachute.timer.get_timer(4)) > parachute.timer.get_duration(4))
		{

			// barometer calculation is now performed in member function
			msl = parachute.baro.get_msl();

			parachute.timer.update_watcher(4); // used to check loop frequency
			parachute.timer.update_timer(4);
		}


		if( (parachute.timer.get_current_time()-parachute.timer.get_timer(5)) > parachute.timer.get_duration(5)){
			// unused timer() loop
			parachute.timer.update_watcher(5); // used to check loop frequency
			parachute.timer.update_timer(5);}

		if( (parachute.timer.get_current_time()-parachute.timer.get_timer(6)) > parachute.timer.get_duration(6)){
			// unused timer() loop
			parachute.timer.update_watcher(6); // used to check loop frequency
			parachute.timer.update_timer(6);}

		if( (parachute.timer.get_current_time()-parachute.timer.get_timer(7)) > parachute.timer.get_duration(7)){

			parachute.timer.update_watcher(7); // used to check loop frequency
			parachute.timer.update_timer(7);}

		if( (parachute.timer.get_current_time()-parachute.timer.get_timer(8)) > parachute.timer.get_duration(8)){
			//Output Message
			//if(dbmsg_global || dbmsg_local){
			if(!dbmsg_global && !dbmsg_local){
				cout << endl;
				cout << "Current filename: " << filename_str << endl;
				cout << "Barometer Altitude: " << msl << " ft" << endl;
				cout << "Raw RC Input:";
				for(int i = 0 ; i < NUM_CHANNELS ; i++ ){
					cout << " channel " << i << ": " << parachute.rc.get_raw(i);
					if(i != NUM_CHANNELS -1){
						cout << ",";}}
				cout << endl;
				cout << "Scaled RC Input:";
				for(int i = 0 ; i < NUM_CHANNELS ; i++ ){
					cout << " channel " << i << ": " << parachute.rc.get_scaled(i);
					if(i != NUM_CHANNELS -1){
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

				piLock(GPSKEY);

				cout << "GPS Status: " << status_gps_string << " GPS Time: " << time_gps << endl;
				cout << "Lat: ";
				cout << setprecision(9) <<  lat;
				cout << " deg\tLng: ";
				cout << setprecision(9) << lng;
				cout << " deg\tAlt(msl): " << msl_gps << " ft\tAlt(ae): " << alt_ellipsoid << endl;
				cout << "Current Waypoint: " << endl;
				cout << "Lat: " << waypoints[wind_level_index][0] << " deg\tLng: " << waypoints[wind_level_index][1] << " def\tAlt(msl): " << waypoints[wind_level_index][2] << " ft\tWind level index: " << wind_level_index << endl;
				cout << "Horz accuracy: " << horz_accuracy << " ft\t"  << " Vert Accuracy: " << vert_accuracy << " ft" << endl;

				piUnlock(GPSKEY);

				cout << "Euler Angles (Mahony): " << "(dt = " << dt << ")" <<endl;
				cout << "MPU9250: Roll: " << roll_mpu_mahony << " Pitch: " << pitch_mpu_mahony << " Yaw: " << yaw_mpu_mahony << endl;
				cout << "LSM9DS1: Roll: " << roll_lsm_mahony << " Pitch: " << pitch_lsm_mahony << " Yaw: " << yaw_lsm_mahony << endl;
				cout << "Euler Angles (Madgwick): " << "(dt = " << dt << ")" << endl;
				cout << "MPU9250: Roll: " << roll_mpu_madgwick << " Pitch: " << pitch_mpu_madgwick << " Yaw: " << yaw_mpu_madgwick << endl;
				cout << "LSM9DS1: Roll: " << roll_lsm_madgwick << " Pitch: " << pitch_lsm_madgwick << " Yaw: " << yaw_lsm_madgwick << endl;

				cout << "Right Winch: " << winch_right_cmd << " Left Winch: " << winch_left_cmd << endl;

				cout << "Control Type: " << control_type_message << endl;

				cout << " PID" << endl << "kp: " << kp << " ki: " << ki << " kd: " << kd << endl;
				cout << "Heading type: " << heading_type_message << endl;
				cout << "Yaw Desired: " << yaw_desired << " Yaw Error: " << yaw_error << " Yaw Error Rate: " << yaw_error_rate << " Yaw Error Sum: " << yaw_error_sum << endl;
				cout << "Proportional: " <<  kp*yaw_error << " Integral: " << ki*yaw_error_sum << " Derivative: " << kd*yaw_error_rate << endl;
				cout << "Number of wraps: " << num_wraps << endl;

				cout << "Yaw Limited Desired: " << yaw_l_desired << endl << endl;

				}

//dbmsg_local = true;
			if(dbmsg_global || dbmsg_local){
				// Alternate debug message, just print out the most current duration for each loop occasionally
				cout << "Printing the most current expected vs. actual time since last execution for each loop" << endl;
				for(int i = 0 ; i < NUM_LOOPS ; i++){
					cout << "For " << parachute.timer.get_frequency(i) << "(Hz) loop, expected: " << parachute.timer.get_duration(i) << "(us), actual: " << parachute.timer.get_watcher(i) << endl;}}
//dbmsg_local = false;

			// Serial Output to help with payload recovery
			if(((int(time_gps)-2) % 10) ==  0){
				cout << "Blasting serial" << endl;
				serialPrintf(serialHandle, "--Payload 2--\nGPS Position:\n");
				serialPrintf(serialHandle, to_string(lat).c_str());
				serialPrintf(serialHandle, "(deg), ");
				serialPrintf(serialHandle, to_string(lng).c_str());
				serialPrintf(serialHandle, "(deg)\nAltitude: ");
				serialPrintf(serialHandle, to_string(msl_gps).c_str());
				serialPrintf(serialHandle, "(ft)\n\n");
			}


			parachute.timer.update_watcher(8); // used to check loop frequency
			parachute.timer.update_timer(8);
		}

	}

	// tidy up and close the file when we exit the inner while loop
	fout.close();
	}
	return 0;
	}
