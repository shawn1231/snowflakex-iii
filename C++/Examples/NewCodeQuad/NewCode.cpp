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

// log file location (relative path)
std::string file_location = "./LogFiles/";

//-----------------------------------------------------------------------------------Read Mag Calibration Using Custom CSV Reader
data_file mpu_offsets("./mpu_mag_cal.csv",4,3);
data_file lsm_offsets("./lsm_mag_cal.csv",4,3);
data_file waypoints_df("./waypoints.csv",1000,3);

// misc junk
wrapped_variable continuous_yaw(360,160);
string filename_str = "";

//---------------------------------------------------------------------------------------------------------Barometer Declarations
// most barometer variables moved to barometer class
float msl = 0.0; // mean sea level altitue (ft) [should be close to 920ft for UMKC Flarsheim]

// rc input declaration moved to remote_control class

//---------------------------------------------------------------------------------------------------------------IMU Declarations
#define DECLINATION 2.30 //magnetic declination for zip 36688 (University of South Alabama) (positive for W, negative for E)
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
#define MOTOR_1_PIN 0 // ID of PWM channel for motor
// careful, physical pin 0 (the first position) is PPM, physical pin 1 (the second position) is PWM position 0 in software
float motor_1_cmd = 0;

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

//Campus Quad Offsets <-- these are default magnetometer calibration values, use with caution
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

// GPS refresh rate is improved by using a thread, refresh could likely be improved but this consistently delivers 1Hz refresh
PI_THREAD (decodeGPS)
{
	(void)piHiPri(99);
	static vector<double> pos_data; // this vector will contain undecoded gps information
	static Ublox gps;
	cout << "Initializing GPS......................." << endl;
	if(!gps.testConnection()){ // check if the gps is working
		cout << "    --ERROR, GPS not initialized--    " << endl;
	}
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


// -------------------
// END SETUP
// -------------------


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

// -------------------
// if you need to assign a specific value to a variable when disarmed, put that assignment here (place 1/3)
// -------------------

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

				// leaving this here, modify/use if you want to read in important parameters without compiling
				/*
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
				*/

				ifs.close(); // close the file
				break;
			// this option reads in a prefix from the program call and uses the defaults for everything else
			case 'd' : logfile_prefix = optarg; logfile_prefix_string = file_location+string(logfile_prefix)+'_'; break;
			// stop execution if option flag is invalid
			case '?' : cout << "Invalid option.  Use -h for help" << endl; return EXIT_FAILURE;}}



	cout << endl << MSG_BAR << endl;
	cout << "         Begin Initialiation           ";
	cout << endl << MSG_BAR << endl;

	// create an object of type vehicle called quad
	vehicle quad;

	quad.timer.update_time();

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
	piThreadCreate(decodeGPS);

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
// -------------------
// Different conditions provided here for different arm mechanisms
// -------------------

		while(!(adc_array[4] < 4000)){ // works with analog voltage (rocker switch) arm mechanism
		//while(!((quad.rc.get_raw(5)>1500)&&(adc_array[4]<4000))){  // works with rocker switch arm mechanism combined with RC Rx
		//bool temp_flag = false; // works with no arm mechanism (always armed) line 1/3 required
		//while(!temp_flag){ // works with no arm mechanism (always armed) line 2/3 required
			if(standby_message_timer > 250){
				cout << endl << "---------------------------------------" << endl << "           Autopilot Inactive         " << endl;
				cout << "  Dynamic Lines at Neutral Deflection " << endl;
				cout << "       Battery Voltage:  " << adc_array[2]/100 << " V" << endl;
				cout << "         Waiting for Killswitch       " << endl << "--------------------------------------" << endl;
				standby_message_timer = 0;}

// -------------------
// if you need to assign a specific value to a variable when disarmed, put that assignment here (place 2/3)
// -------------------
			// for example, good idea to set motors to not spin when we are disarmed
			quad.pwm_out.set_duty_cycle(MOTOR_1_PIN, 1.0);

			// update RC array (for arm condition)
			quad.rc.update();
			// update adc array (for arm condition)
			adc_array[4] = adc.read(4);

			usleep(5000);
			standby_message_timer++; // increment the message delay timer
			//temp_flag = true; // works with no arm mechanism (always armed) line 3/3 required

		}

		// code below uses the time on the Navio as the filename
		// !!! Since RaspberryPi has no real time clock this time is only accurate under two conditions:
		// 1)  time is manually set on client or polled from manually configured time server
		// 2)  RaspberryPi has internet connection and can use internet time servers
		time_t result = time(NULL);
		char *today = asctime(localtime(&result));
		cout << filename_str << endl;
		filename_str = generate_filename(logfile_prefix_string);

		// create the file output stream object
		ofstream fout;
		// open the file
		cout << "Log file created at: " << endl << filename_str << endl << endl;
		fout.open(filename_str,ios::out);

// -------------------
// reformat this header string based on what data you want to log
// header string is not ties to the data being logged in anyway, it is up to the programmer to ensure
// that the header string and the data being logged match, always double check an example csv after
// making changes because a misplaced comma or name in the wrong position can make the resulting data
// files unreadable
// -------------------

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
			"motor1cmd,time_gps,lat,lng,alt_ellipsoid,msl_gps,"
			"horz_accuracy,vert_accuracy,status_gps"
			<< endl;
		usleep(20000);


// -------------------
// if you need to assign a specific value to a variable when disarmed, put that assignment here (place 3/3)
// NOTE there are differences between these three places where you can do "one time" variable assignment but
// I will not explain them here.  If you need more control over the specific behavior then feel free to
// experiment and/or look into the code.  For most applications just copy/paste the same code in all three
// places and the desired behavior will result.
// -------------------


	while(adc_array[4] < 4000) // for analog voltage (rocker swith) arm mechanism
	//while((quad.rc.get_raw(5)>1500)&&(adc_array[4]<4000)) // rocker switch + RC rx arm mechanism
	//while(true) // no arm switch (always armed)
	{

		quad.timer.update_time();

		if( (quad.timer.get_current_time()-quad.timer.get_timer(0)) > quad.timer.get_duration(0))
		{
			//----------------------------------------------------------------------------------------------------------------AHRS Update
			dt = quad.timer.get_current_time()-quad.timer.get_timer(0);
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

			quad.timer.update_watcher(0); // used to check loop frequency
			quad.timer.update_timer(0);
		}

		if( (quad.timer.get_current_time()-quad.timer.get_timer(1)) > quad.timer.get_duration(1))
		{
			// unused timer loop
			quad.timer.update_watcher(1); // used to check loop frequency
			quad.timer.update_timer(1);
			}

		if( (quad.timer.get_current_time()-quad.timer.get_timer(2)) > quad.timer.get_duration(2))
		{

			// unused timer loop
			quad.timer.update_watcher(2); // used to check loop frequency
			quad.timer.update_timer(2);
		}

		if( (quad.timer.get_current_time()-quad.timer.get_timer(3)) > quad.timer.get_duration(3))
		{

			// even though GPS is threaded, we have to decode the gps message and store it in a local variable
			piLock(GPSKEY);

			// NOTE:  we have had issues with slow GPS refresh rate, it seems like disabling the message decoding
			// alleviates these problems, look here first if GPS seems abnormally slow, uncommenting this decode
			// block will cause GPS status to always say "no fix"
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
			quad.baro.update_sensor();

			// all rc reading is accomplished by member function from remote_control class
			quad.rc.update();

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

// -------------------
// Do interesting stuff here, change motor_1_cmd based on a controller, etc.
// -------------------

			// always write the duty cycle
			quad.pwm_out.set_duty_cycle(MOTOR_1_PIN, motor_1_cmd);

			//-------------------------------------------------------------------------------------------------------Data Log File Output
			fout << today << "," << quad.timer.get_current_time() << "," << msl << ",";
			for(int i = 0 ; i < NUM_CHANNELS ; i++){
				fout << quad.rc.get_raw(i) << ",";}
			for(int i = 0 ; i < NUM_CHANNELS ; i++){
				fout << quad.rc.get_scaled(i) << ",";}
			for(int i = 0 ; i < ARRAY_SIZE(adc_array) ; i++){
				fout << adc_array[i] << ",";}
			fout << roll_mpu_mahony << "," << pitch_mpu_mahony << "," << yaw_mpu_mahony << ",";
			fout << roll_lsm_mahony << "," << pitch_lsm_mahony << "," << yaw_lsm_mahony << ",";
			fout << roll_mpu_madgwick << "," << pitch_mpu_madgwick << "," << yaw_mpu_madgwick << ",";
			fout << roll_lsm_madgwick << "," << pitch_lsm_madgwick << "," << yaw_lsm_madgwick << ",";
			fout << a_mpu[0] << "," << a_mpu[1] << "," << a_mpu[2] << "," << a_lsm[0] << "," << a_lsm[1] << ",";
			fout << a_lsm[2] << "," << g_mpu[0] << "," << g_mpu[1] << "," << g_mpu[2] << "," << g_lsm[0] << "," << g_lsm[1] << ",";
			fout << g_lsm[2] << "," << m_mpu[0] << "," << m_mpu[1] << "," << m_mpu[2] << "," << m_lsm[0] << "," << m_lsm[1] << "," << m_lsm[2] << ",";
			fout << motor_1_cmd << "," ;

			// have not rigorously tested whether the piLock is neccesary here, doesn't seem like it is probably needed but I'm keeping
			// it until I have a chance to do more comprehensive testing
			piLock(GPSKEY);
			fout << setprecision(9) << time_gps << "," << lat << "," << lng << "," << alt_ellipsoid << "," << msl_gps << "," << horz_accuracy << "," << vert_accuracy << "," << status_gps << "," << status_gps_string << ",";
			piUnlock(GPSKEY);

			fout << endl;



			quad.timer.update_watcher(3); // used to check loop frequency
			quad.timer.update_timer(3);
		}

		if( (quad.timer.get_current_time()-quad.timer.get_timer(4)) > quad.timer.get_duration(4))
		{

			// barometer calculation is now performed in member function
			msl = quad.baro.get_msl();

			quad.timer.update_watcher(4); // used to check loop frequency
			quad.timer.update_timer(4);
		}


		if( (quad.timer.get_current_time()-quad.timer.get_timer(5)) > quad.timer.get_duration(5)){
			// unused timer() loop
			quad.timer.update_watcher(5); // used to check loop frequency
			quad.timer.update_timer(5);}

		if( (quad.timer.get_current_time()-quad.timer.get_timer(6)) > quad.timer.get_duration(6)){
			// unused timer() loop
			quad.timer.update_watcher(6); // used to check loop frequency
			quad.timer.update_timer(6);}

		if( (quad.timer.get_current_time()-quad.timer.get_timer(7)) > quad.timer.get_duration(7)){

			quad.timer.update_watcher(7); // used to check loop frequency
			quad.timer.update_timer(7);}

		if( (quad.timer.get_current_time()-quad.timer.get_timer(8)) > quad.timer.get_duration(8)){

			if(1)
			{
				cout << endl;
				cout << "Current filename: " << filename_str << endl;
				cout << "Barometer Altitude: " << msl << " ft" << endl;
				cout << "Raw RC Input:";
				for(int i = 0 ; i < NUM_CHANNELS ; i++ ){
					cout << " channel " << i << ": " << quad.rc.get_raw(i);
					if(i != NUM_CHANNELS -1){
						cout << ",";}}
				cout << endl;
				cout << "Scaled RC Input:";
				for(int i = 0 ; i < NUM_CHANNELS ; i++ ){
					cout << " channel " << i << ": " << quad.rc.get_scaled(i);
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
				cout << "Horz accuracy: " << horz_accuracy << " ft\t"  << " Vert Accuracy: " << vert_accuracy << " ft" << endl;

				piUnlock(GPSKEY);

				cout << "Euler Angles (Mahony): " << "(dt = " << dt << ")" <<endl;
				cout << "MPU9250: Roll: " << roll_mpu_mahony << " Pitch: " << pitch_mpu_mahony << " Yaw: " << yaw_mpu_mahony << endl;
				cout << "LSM9DS1: Roll: " << roll_lsm_mahony << " Pitch: " << pitch_lsm_mahony << " Yaw: " << yaw_lsm_mahony << endl;
				cout << "Euler Angles (Madgwick): " << "(dt = " << dt << ")" << endl;
				cout << "MPU9250: Roll: " << roll_mpu_madgwick << " Pitch: " << pitch_mpu_madgwick << " Yaw: " << yaw_mpu_madgwick << endl;
				cout << "LSM9DS1: Roll: " << roll_lsm_madgwick << " Pitch: " << pitch_lsm_madgwick << " Yaw: " << yaw_lsm_madgwick << endl;

				cout << "Motor 1: " << motor_1_cmd << endl;

			}

			if(0)
			{
				// Alternate debug message, just print out the most current duration for each loop occasionally
				cout << "Printing the most current expected vs. actual time since last execution for each loop" << endl;
				for(int i = 0 ; i < NUM_LOOPS ; i++){
					cout << "For " << quad.timer.get_frequency(i) << "(Hz) loop, expected: " << quad.timer.get_duration(i) << "(us), actual: " << quad.timer.get_watcher(i) << endl;}
			}


			quad.timer.update_watcher(8); // used to check loop frequency
			quad.timer.update_timer(8);
		}

	}

	// tidy up and close the file when we exit the inner while loop
	fout.close();
	}
	return 0;
	}
