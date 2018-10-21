#ifndef DATA_FILEH_
#define DATA_FILEH_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

class data_file
{
	private:
		std::string file_path; // path relative to current location (could also be absolute)
		std::ifstream ifs;
		int max_col;
		int max_row;
		char delimiter;
		std::vector<std::vector<float>> data;
		void read_file();

	public:
		data_file();
		data_file(std::string des_file_path, int row_to_read, int col_to_read);
		data_file(std::string des_file_path, int row_to_read, int col_to_read, char des_delimiter);
		void print_data();
		void print_one(int r, int c);
		void column2array(float * out_var, int start_column);
		void row2array(float * out_var, int start_row);
		// void mat2array(float * out_var, int start_row, int start_col, int num_rows, int num_cols);
};

#endif

