#include "barometer.h"
#include <iostream>

barometer::barometer()
{
	std::cout << "Initializing barometer................." << std::endl;
	baro_sensor.initialize();
	std::cout << " --Barometer successfully initalized-- " << std::endl;
}

void barometer::update_sensor()
{
	//-------------------------------------------------------------------------------------------------------------Barometer Read
	if(baro_step == 0){
		baro_sensor.refreshPressure();}
	else if(baro_step == 1){
		baro_sensor.readPressure();}
	else if(baro_step == 2){
		baro_sensor.refreshTemperature();}
	else if(baro_step == 3){
		baro_sensor.readTemperature();}
	else {
		baro_sensor.calculatePressureAndTemperature();
		baro_step = -1;
		//-------------------------------------------------------------------------------------------------------------Barometer Calc
		// barometer does full update at 25Hz, no need to do the full altitude calculation any faster than 50Hz
		Tc  = baro_sensor.getTemperature(); // temperature from sensor (C), value is recorded at surface of PCB, (higher than ambient!)
		Tk  = Tc + 273.15; // convert to Kelvin (needed for barometric formula
		Tf  = Tc * (9.0/5.0) + 32; // convert to Fahrenheit (uncomment for sanity check but it is not used for any calcs)
		Pm  = baro_sensor.getPressure(); // pressure is natively given in mbar
		Phg = Pm*.02953; // convert to inHg
		// barometric equation
		m_msl = hb + (Tb/Lb)*(pow((Phg/Pb),((-R*Lb)/(g0*M)))-1); // NOTE: using local Tk instead of standard temperature (Tb)
	}
	baro_step++;
}

float barometer::get_msl()
{
	return m_msl;
}
