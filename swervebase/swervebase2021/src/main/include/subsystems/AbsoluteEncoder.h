
#pragma once

#include <frc/AnalogInput.h>
#include <wpi/math>

class AbsoluteEncoder : public frc::AnalogInput {

	bool flipped = false;	

 public:
	
	AbsoluteEncoder(int channel, bool flip=false):AnalogInput(channel) {
    	flipped = flip;
	};
	
    double GetPosition(){
    	double angle;
    	double voltage = GetVoltage();
    	if (voltage > 3.3) voltage = 3.3;
    	if (flipped) angle = (3.3 - voltage) * (2.0*wpi::math::pi) / 3.3;
    	else angle = voltage * (2.0*wpi::math::pi) / 3.3;

    	return angle; //- wpi::math::pi;
	};

};
