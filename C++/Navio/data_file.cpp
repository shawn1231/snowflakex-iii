#include "data_file.h"

void data_file::read_file()
{
	char trash;
	ifs.open(file_path);
	for(int i = 0 ; i < max_row ; i++) // this is the row iterator
	{
		for(int j = 0 ; j < max_col ; j++ )
		{
			if(j != 0) // first column has no leading delimiter
			{
				ifs >> trash;  // read the delimiter, it will not be used
			}
			else
			{
				// do nothing, no delimiter to remove
			}
			// always do these things
			ifs >> data[j][i];
		}
	}
	ifs.close();
}

data_file::data_file() // blank constructor, useless
{
	file_path = "null";
	max_col = 0;
	max_row = 0;
	char delimiter = '0';
	data.resize(max_col);
	for( int i = 0 ; i++ ; i< max_col)
	{
		data[i].resize(max_row);
	}
	// don't read file, there is nothing to read
}

data_file::data_file(std::string des_file_path, int row_to_read, int col_to_read) // useful constructor
{
	file_path = des_file_path;
	max_col = col_to_read;
	max_row = row_to_read;
	char delimiter = ','; // default to comma
	data.resize(max_col);
	for( int i = 0 ; i< max_col ; i++ )
	{
		data[i].resize(max_row);
	}
	read_file();
}


data_file::data_file(std::string des_file_path, int row_to_read, int col_to_read, char des_delim) // useful constructor
{
	file_path = des_file_path;
	max_col = col_to_read;
	max_row = row_to_read;
	char delimiter = des_delim;
	data.resize(max_col);
	for( int i = 0 ; i++ ; i< max_col)
	{
		data[i].resize(max_row);
	}
	read_file();
}

void data_file::print_data()
{
	for(int i = 0 ; i < max_row ; i++) // this is the row iterator
	{
		for(int j = 0 ; j < max_col ; j++ )
		{
			if(j != 0) // first column has no leading delimiter
			{
				std::cout << ",";
			}
			else
			{
				// do nothing, no delimiter to remove
			}
			// always do these things
			// indexing seems backwards but this makes the expected printout
			std::cout << data[j][i];
		}
		std::cout << std::endl;
	}
}

void data_file::print_one(int r, int c)
{
	std::cout << data[c][r] << std::endl;
}

void data_file::column2array(float * out_var, int start_column)
{
	start_column--;
	for( int row_index = 0 ; row_index < max_row ; row_index++ )
	{
		out_var[row_index] = data[start_column][row_index];
	}
}

void data_file::row2array(float * out_var, int start_row)
{
	start_row--;
	for( int col_index = 0 ; col_index < max_col ; col_index++ )
	{
		out_var[col_index] = data[col_index][start_row];
	}
}

/* void data_file::mat2array(float * out_var, int start_row, int start_cols, int num_rows, int num_cols)
{

	start_row--;
	start_col--;
	
	// make sure the limits are valid, if not set them to the maximum allowable rows/cols
	if( !(num_rows + start_row < max_row) )
	{
		max_rows = max_row;
	} else
	{
		max_rows = num_rows;
	}
	if( !(num_cols + start_col < max_col) )
	{
		max_cols = max_col;
	} else
	{
		max_cols = num_cols;
	}
	
	
	for( int col_index = start_col ; col_index < max_cols ; col_index++ )
	{
		for( int row_index = start_row ; row_index < max_rows ; row_index++ )
		{
			out_var[col_index][row_index] = data[col_index][row_index];
		}
	}
} */
