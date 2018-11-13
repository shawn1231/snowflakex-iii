#ifndef _DIGITAL_FILTERH_
#define _DIGITAL_FILTERH_

#include <cmath>
#include <iostream>
#include "buffer.h"

using namespace std;

class digital_filter
{
		vector<float> a;
		vector<float> b;
		int order;
		float fc, fs;
		char type;
		buffer input;
		buffer output;
	public:
		digital_filter(int,char,float,float);
		float filter_new_input(float);
//		float get_current_out();
		void show();
		void description();
};

digital_filter::digital_filter(int des_order, char des_type, float des_fc, float des_fs)
{
	order = des_order;
	fc = des_fc;
	fs = des_fs;
	type = des_type;

	input.resize(order+1);
	output.resize(order+1);

	// fill with zeros (seed the buffers)
	for(int i = 0 ; i < order+1 ; i++)
	{
		input.put(0.0);
		output.put(0.0);
	}

	float gamma;
	float D;

	switch(type)
	{
		case 'l': // low pass option
		{
			gamma = tan((3.14*fc)/fs);
			cout << "Low pass filter selected" << endl;
			switch(order)
			{
				case 1: // 1st order
				{
					cout << "1st order filter selected" << endl;
					D = gamma+1.0;
					a.resize(order+1);
					b.resize(order+1);
					b[0] = gamma;
					b[1] = b[0];
					b[0] = b[0]/D;
					b[1] = b[1]/D;
					a[1] = (gamma-1)/D;
					break;
				}
				case 2: // 2nd order
				{
					cout << "2nd order filter selected" << endl;
					D = pow(gamma,2.0) + pow(2.0,(1.0/2.0))*gamma+1.0;
					a.resize(order+1);
					b.resize(order+1);
					b[0] = pow(gamma,2.0);
					b[1] = 2.0*b[0];
					b[2] = b[0];
					b[0] = b[0]/D;
					b[1] = b[1]/D;
					b[2] = b[2]/D;
					a[1] = (2.0*(pow(gamma,2.0)-1.0))/D;
					a[2] = (pow(gamma,2.0)-pow(2.0,(1.0/2.0))*gamma+1.0)/D;
					break;
				}
				case 3: // 3rd order
				{
					cout << "3rd order filter selected" << endl;
					D = pow(gamma,3.0)+2.0*pow(gamma,2.0)+2.0*gamma+1.0;
					a.resize(order+1);
					b.resize(order+1);
					b[0] = pow(gamma,3.0);
					b[1] = 3.0*b[0];
					b[2] = 3.0*b[0];
					b[3] = b[0];
					b[0] = b[0]/D;
					b[1] = b[1]/D;
					b[2] = b[2]/D;
					b[3] = b[3]/D;
					a[1] = (3.0*pow(gamma,3.0)+2.0*pow(gamma,2.0)-2.0*gamma-3.0)/D;
					a[2] = (3.0*pow(gamma,3.0)-2.0*pow(gamma,2.0)-2.0*gamma+3.0)/D;
					a[3] = (pow(gamma,3.0)-2.0*pow(gamma,2.0)+2.0*gamma-1.0)/D;
					break;
				}
				case 4: // 4th order
				{
					cout << "4th order filter selected" << endl;
					a.resize(order+1);
					b.resize(order+1);
					float alpha = -2.0*(cos((5.0*3.14)/8.0)+cos((7.0*3.14)/8.0));
					float beta = 2.0*(1.0+2.0*cos((5.0*3.14)/8.0)*cos((7.0*3.14)/8.0));
					D = pow(gamma,4.0)+alpha*pow(gamma,3.0)+beta*pow(gamma,2.0)+alpha*gamma+1.0;
					b[0] = pow(gamma,4);
					b[1] = 4.0*b[0];
					b[2] = 6.0*b[0];
					b[3] = 4.0*b[0];
					b[4] = b[0];
					b[0] = b[0]/D;
					b[1] = b[1]/D;
					b[2] = b[2]/D;
					b[3] = b[3]/D;
					b[4] = b[4]/D;
					a[1] = (2.0*(2.0*pow(gamma,4.0)+alpha*pow(gamma,3.0)-alpha*gamma-2.0))/D;
					a[2] = (2.0*(3.0*pow(gamma,4.0)-beta*pow(gamma,2.0)+3.0))/D;
					a[3] = (2.0*(2.0*pow(gamma,4.0)-alpha*pow(gamma,3.0)+alpha*gamma-2.0))/D;
					a[4] = (pow(gamma,4.0) - alpha*pow(gamma,3.0)+beta*pow(gamma,2.0)-alpha*gamma+1.0)/D;
					break;
				}}
			break;
		}
		case 'h':
		{
			gamma = tan((3.14*fc)/fs);
			cout << "High pass filter selected" << endl;
			switch(order)
			{

				case 1: // 1st order
				{
					cout << "1st order filter selected" << endl;
					D = gamma+1.0;
					a.resize(order+1);
					b.resize(order+1);
					b[0] = 1.0;
					b[1] = -1.0;
					b[0] = b[0]/D;
					b[1] = b[1]/D;
					a[1] = (gamma-1.0)/D;
					break;
				}
				case 2: // 2nd order
				{
					cout << "2nd order filter selected" << endl;
					D = pow(gamma,2.0) + pow(2.0,(1.0/2.0))*gamma+1.0;
					a.resize(order+1);
					b.resize(order+1);
					b[0] = 1.0;
					b[1] = -2.0;
					b[2] = 1.0;
					b[0] = b[0]/D;
					b[1] = b[1]/D;
					b[2] = b[2]/D;
					a[1] = (2.0*(pow(gamma,2.0)-1.0))/D;
					a[2] = (pow(gamma,2.0)-pow(2.0,(1.0/2.0)*gamma+1.0))/D;
					break;
				}
				case 3: // 3rd order
				{
					cout << "3rd order filter selected" << endl;
					D = pow(gamma,3.0)+2.0*pow(gamma,2.0)+2.0*gamma+1.0;
					a.resize(order+1);
					b.resize(order+1);
					b[0] = 1.0;
					b[1] = -3.0;
					b[2] = 3.0;
					b[3] = -1.0;
					b[0] = b[0]/D;
					b[1] = b[1]/D;
					b[2] = b[2]/D;
					b[3] = b[3]/D;
					a[1] = (3.0*pow(gamma,3.0)+2.0*pow(gamma,2.0)-2.0*gamma-3.0)/D;
					a[2] = (3.0*pow(gamma,3.0)-2.0*pow(gamma,2.0)-2.0*gamma+3.0)/D;
					a[3] = (pow(gamma,3.0)-2.0*pow(gamma,2.0)+2.0*gamma-1.0)/D;
					break;
				}
				case 4: // 4th order
				{
					cout << "4th order filter selected" << endl;
					a.resize(order+1);
					b.resize(order+1);
					float alpha = -2.*(cos((5.0*3.14)/8.0)+cos((7.0*3.14)/8.0));
					float beta = 2.0*(1.0+2.0*cos((5.0*3.14)/8.0)*cos((7.0*3.14)/8.0));
					D = pow(gamma,4.0)+alpha*pow(gamma,3.0)+beta*pow(gamma,2.0)+alpha*gamma+1.0;
					b[0] = 1.0;
					b[1] = -4.0;
					b[2] = 6.0;
					b[3] = -4.0;
					b[4] = 1.0;
					b[0] = b[0]/D;
					b[1] = b[1]/D;
					b[2] = b[2]/D;
					b[3] = b[3]/D;
					b[4] = b[4]/D;
					a[1] = (2.0*(2.0*pow(gamma,4.0)+alpha*pow(gamma,3.0)-alpha*gamma-2.0))/D;
					a[2] = (2.0*(3.0*pow(gamma,4.0)-beta*pow(gamma,2.0)+3.0))/D;
					a[3] = (2.0*(2.0*pow(gamma,4.0)-alpha*pow(gamma,3.0)+alpha*gamma-2.0))/D;
					a[4] = (pow(gamma,4.0) - alpha*pow(gamma,3.0)+beta*pow(gamma,2.0)-alpha*gamma+1.0)/D;
					break;
				}
				default:
				{
					cout << "filter order not supported" << endl;
					break;
				}
				break;
			}}
		default:
		{
			cout << "filter type not supported" << endl;
			break;
		}
	}
	return;
}

void digital_filter::show()
{
	for( int i = 1 ; i < order+1 ; i++ )
	{
		cout << "a[" << i << "] = " << a[i] << " ";
	}
	cout << endl;
	for( int i = 0 ; i< order+1 ; i++ )
	{
		cout << "b[" << i << "] = " << b[i] << " ";
	}
	cout << endl;
//	return;
}

float digital_filter::filter_new_input(float input_val)
{
	float new_out = 0;
	input.put(input_val);

	for(int i = 0; i < order + 1; i ++ )
	{
		new_out = new_out + b[i]*input.get(i);
	}
	for(int i = 0; i < order ; i ++)
	{
		new_out = new_out + (-a[i+1])*output.get(i);
	}
	output.put(new_out);

	return new_out;
}

//float digital_filter::get_current_out()
//{
//	return output.get(0);
//}


void digital_filter::description()
{
	cout << "I am a " << type << "p filter of order " << order << endl;
	cout << "I am based on a sampling frequency of " << fs << "Hz" << endl;
	cout << "and my cutoff frequency is " << fc << "Hz" << endl;
//	return;
}

#endif
