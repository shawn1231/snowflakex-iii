#ifndef WRAPPED_VARIABLEH_
#define WRAPPED_VARIABLEH_

class wrapped_variable
{
	public:
		wrapped_variable();
		wrapped_variable(float des_wrap_size, float des_threshold);
		float process_new_input(float current_original);
		float get_current_continuous();
		float get_number_of_wraps();
		void reset_wrap_counter();

	private:
		float original;
		float continuous;
		int number_of_wraps;
		float wrap_size;
		float threshold;
};


#endif