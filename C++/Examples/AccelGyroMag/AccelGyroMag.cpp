clear
/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Read accelerometer, gyroscope and magnetometer values from
inertial measurement unit: MPU9250 or LSM9DS1 over SPI on Raspberry Pi + Navio.

Navio's onboard sensors are connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1 (MPU9250), /dev/spidev0.3 (acc/gyro LSM9DS1)
and /dev/spidev0.2 (mag LSM9DS1).
*/


//#include "Navio/NewClass.h"
#include "Navio/MPU9250.h"
#include "Navio/LSM9DS1.h"
#include "Navio/Util.h"
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <sys/time.h>
#include <ctime>
#include <string>

using namespace std;

#define TITLE_LENGTH 12
#define OFFSET 4

char file_title[TITLE_LENGTH+5];
struct timeval tv;
long long currenttime = 0;
long long offset = 0;

bool file_exists(string filename)
{
	ifstream checkfile(filename);
	return checkfile;
}

int main( int argc, char *argv[] )
{

    int parameter;
    char *logfile_notes;
    string file_path;

    while((parameter = getopt(argc,argv, "hd:")) != -1){
	switch(parameter){
		case 'h': cout << "Use option \"-d <FileNotes>\" to append a note to the log file" << endl; return EXIT_FAILURE;
		case 'd': logfile_notes = optarg;
		    file_path = string(logfile_notes);
		    file_path = "LogFiles/"+file_path+"_";
		    break;
		case '?': cout << "Wrong options" << endl; return EXIT_FAILURE;}}
    //cout << logfile_path << endl;


    time_t result = time(NULL);
    char *today = asctime(localtime(&result));
    today[strlen(today) - 1]= '\0';
    for(int i = 0 ; i < TITLE_LENGTH ; i++){
	if(today[i+OFFSET] == ' ' || today[i+OFFSET] == ':'){
		if(i == 8-OFFSET){
//			cout << "add leading zero" << endl;
			file_title[i] = '0';}
		else{
//			cout << "caught the chars" << endl;
			file_title[i] = '_';}}
	else{
//		cout << "else condition" << endl;
		file_title[i] = today[i+OFFSET];}}

    //cout << endl << file_title << endl;

    InertialSensor *sensor;
    sensor = new MPU9250();

//    file_title[TITLE_LENGTH+1] = '_';
    file_title[TITLE_LENGTH+0] = '-';
    file_title[TITLE_LENGTH+1] = '0';
    file_title[TITLE_LENGTH+2] = '.';
    file_title[TITLE_LENGTH+3] = 'c';
    file_title[TITLE_LENGTH+4] = 's';
    file_title[TITLE_LENGTH+5] = 'v';

//    string file_path(logfile_notes);
//    file_path = "LogFiles/"+file_path+"_";
//    cout << "this is the file path to be added" << file_path << endl;

    string file_title_str(file_title);
    file_title_str = file_path+file_title_str;
    cout<< "This is the whole file title: " << endl << file_title_str << endl;

    int file_title_index = 1;



    while(file_exists(file_title_str))
    {
	//cout << "cannot create file" << file_title << endl;
	file_title[TITLE_LENGTH+0] = '-';
	file_title[TITLE_LENGTH+1] = file_title_index+'0';
	file_title[TITLE_LENGTH+2] = '.';
	file_title[TITLE_LENGTH+3] = 'c';
	file_title[TITLE_LENGTH+4] = 's';
	file_title[TITLE_LENGTH+5] = 'v';
	file_title_index++;
	string temp(file_title);
	file_title_str = file_path+temp;
	usleep(50000);
    }

    //string str(file_title);

    cout << file_title_str << endl;
    ofstream fout;
    fout.open(file_title_str,ios::out);

   // fout << "start_time,time_since_start,ax,ay,az,gx,gy,gz,mx,my,mz" << endl;

    sensor->initialize();

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
//-------------------------------------------------------------------------

    while(1) {
        sensor->update();
        sensor->read_accelerometer(&ax, &ay, &az);
        sensor->read_gyroscope(&gx, &gy, &gz);
        sensor->read_magnetometer(&mx, &my, &mz);
        printf("Acc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
        printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
        printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);

	gettimeofday(&tv, NULL);
	if(offset == 0)
	{
		offset = 1000000 * tv.tv_sec + tv.tv_usec;
	}

	currenttime = (1000000 * tv.tv_sec + tv.tv_usec)-offset;

//	cout << currenttime << ", " << offset << ",";

	fout << today << "," << currenttime << "," << ax << "," << ay << "," << az << "," << gx << "," << gy << "," << gz << "," << mx << "," << my << "," << mz << endl;
       usleep(500000);
    }
    return 0;
}
