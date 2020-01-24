/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/WheelOfFortune.h"


WheelOfFortune::WheelOfFortune() {

}

void WheelOfFortune::Periodic() {
  // Implementation of subsystem periodic method goes here.
  
}

void WheelOfFortune::SpinToColorInit(){
  frc::SmartDashboard::PutString("Color Defined", "R");
  Goal = frc::SmartDashboard::GetString("Color Defined", "R");
  m_motor.Set(1);
  done = false;
}

void WheelOfFortune::SpinToColor(){
  frc::Color detectedColor = m_colorSensor.GetColor();

    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);

  if ( detectedColor.red > 0.4 && detectedColor.green < 0.4 && detectedColor.blue < 0.4) {
    Current = "R";
  }
  if ( detectedColor.red < 0.4 && detectedColor.green > 0.55 && detectedColor.blue < 0.4) {
    Current = "G";
  }
  if ( detectedColor.red < 0.4 && detectedColor.green < 0.5 && detectedColor.blue > 0.3) {
    Current = "B";
  }
  if ( detectedColor.red > 0.4 && detectedColor.green > 0.4 && detectedColor.blue < 0.3) {
    Current = "Y";
  }

    if (Goal == "Y" && Current == "G") {
      m_motor.Set(0);
      done = true;
    }
    if (Goal == "B" && Current == "R") {
      m_motor.Set(0);
      done = true;
    }
    if (Goal == "G" && Current == "Y") {
      m_motor.Set(0);
      done = true;
    }
    if (Goal == "R" && Current == "B") {
      m_motor.Set(0);
      done = true;
    }
}

void WheelOfFortune::SpinWheel3Init(){
  RedCount = 0;
  onRed = false;
  m_motor.Set(1);
  done = false;
}

void WheelOfFortune::SpinWheel3(){
  frc::Color detectedColor = m_colorSensor.GetColor();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);

  if(detectedColor.red > 0.4 && detectedColor.green < 0.4 && detectedColor.blue < 0.4 && onRed == false) {
        RedCount++;
        onRed = true;
  } else if(detectedColor.red < 0.4 && detectedColor.green > 0.55 && detectedColor.blue < 0.4) {
        onRed = false;
  } else if (detectedColor.red < 0.4 && detectedColor.green < 0.5 && detectedColor.blue > 0.3) {
        onRed = false;
  } else if (detectedColor.red > 0.4 && detectedColor.green > 0.4 && detectedColor.blue < 0.3) {
        onRed = false;
  }
      if(RedCount >= 7) {
        m_motor.Set(0);
        done = true;
      }
      
      std::cout << "RedCount: " << RedCount << "\n";
}

bool WheelOfFortune::IsDone(){
  return done;
}