#ifndef _BUFFERH_
#define _BUFFERH_

#include <vector>
#include <iostream>

using namespace std;

class buffer{
		int length;
		vector<float> array;
	public:
		buffer();
		buffer(int);
		void put(float);
		float get(int);
		void show();
		void resize(int);
};

buffer::buffer()
{
	length = 1;
	array.resize(1);
}

buffer::buffer(int size)
{
	if(size < 2)
	{
		cout << "too small" << endl;
		return;
	}
	length = size;
	array.resize(size);
}

void buffer::put(float input_val)
{
//	cout << "put is working " << endl;
	for(int i = length-1 ; i >= 0 ; i--)
	{
		array[i] = array[i-1];
//		cout << i << " " << array[i] << " " << array[i-1] << endl;
	}
	array[0] = input_val;
	return;
}

float buffer::get(int index)
{
	return array[index];
}

void buffer::show()
{
//	cout << "show is working" << endl;
	for(int i = 0 ; i < length ; i ++)
	{
		cout  << array[i] << "\t ";
//		cout << i << endl;
	}
	cout << endl;
}

void buffer::resize(int size)
{
	length = size;
	array.resize(size);
}

#endif
