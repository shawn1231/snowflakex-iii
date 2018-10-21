#ifndef _HERRINGTON_UTILSH_
#define _HERRINGTON_UTILSH_

#include <cstring>
#include <string>
#include <iostream>
#include <sys/time.h>
#include <unistd.h>

float saturate(float input, float limit)
{
	float output;
	if(input > limit)
	{
		output = limit;
	}
	else if(input < -limit)
	{
		output = -limit;
	}
}

// function to check if file exists so that the filename can be dynamically changed
bool file_exists(std::string name_of_file)
{
	std::ifstream checkfile(name_of_file); // try to read from the file
	return bool(checkfile); // cast the result to bool, if file opens, this returns true (it exists)
}

std::string generate_filename(std::string logfile_prefix_string)
{
	// these are used to format the filename string
	#define FILENAME_LENGTH 12
	#define FILENAME_OFFSET 4
	// leave extra room for '.csv' and potential '-1', '-2', etc.
	char filename[FILENAME_LENGTH+6];
	
	time_t result = time(NULL);
	char *today = asctime(localtime(&result)); // establish char array to write today's date
	today[std::strlen(today) - 1] = '\0'; // remove end line from the end of the character array
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
	filename[FILENAME_LENGTH+6] = 0;
	// filename[FILENAME_LENGTH+6] = 0;
	// for adding the relative path to the file, it will be converted to a string
	std::string filename_str(filename);
	filename_str = logfile_prefix_string+filename_str;
	// used to put a number on the end of the file if it is a duplicate
	int filename_index = 1;
	// loop while the file name already exists or until we are out of indices to write to
	std::cout << "Checking for unique file name.........." << std::endl;
	while(file_exists(filename_str))
	{
	std::cout << "Filename already exists................" << std::endl;
		filename[FILENAME_LENGTH+0] = '-';
		filename[FILENAME_LENGTH+1] = filename_index + '0';
		filename[FILENAME_LENGTH+2] = '.';
		filename[FILENAME_LENGTH+3] = 'c';
		filename[FILENAME_LENGTH+4] = 's';
		filename[FILENAME_LENGTH+5] = 'v';
		filename[FILENAME_LENGTH+6] = 0;
		std::cout << filename_str << std::endl;
		std::string temp(filename);
		filename_str = logfile_prefix_string+temp;
		std::cout << filename_str << std::endl;
		filename_index++;
		usleep(50000);} // don't try to read the files at warp speed
		std::cout << filename_str << std::endl;
		return filename_str;
}


#endif
