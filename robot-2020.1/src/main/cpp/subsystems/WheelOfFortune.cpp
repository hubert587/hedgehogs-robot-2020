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
int WheelOfFortune::GetColor(){
  frc::Color detectedColor = m_colorSensor.GetColor();
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  double tolerance = frc::SmartDashboard::GetNumber("Tolerance", .8);
  rev::ColorMatch Matcher;
  for (int x = 0; x < 4; x++) {
    Matcher.AddColorMatch(ColorConstants::kColorCodes[x]);
  }
  Matcher.SetConfidenceThreshold(tolerance);
  std::optional<frc::Color> matchedColor = Matcher.MatchColor(detectedColor);
  frc::SmartDashboard::PutNumber("Matched Red", matchedColor.has_value() ? matchedColor.value().red : 0);
  frc::SmartDashboard::PutNumber("Matched Green", matchedColor.has_value() ? matchedColor.value().green : 0);
  frc::SmartDashboard::PutNumber("Matched Blue", matchedColor.has_value() ? matchedColor.value().blue : 0);
  int colorIndex = -1;
  for (int color = 0; color < 4; color++) {
    if (matchedColor == ColorConstants::kColorCodes[color]) {
      frc::SmartDashboard::PutNumber("Color", color);
      colorIndex = color;
    }
  }
  return colorIndex;
}

std::string WheelOfFortune::ConvertColor(int colorIndex){
  if (colorIndex == 0) {
    return "G";
  } else if (colorIndex == 1) {
    return "R";
  }  else if (colorIndex == 2) {
    return "B";
  }  else if (colorIndex == 3) {
    return "Y";
  } else {
    return "Unknown";
  }
  
}

std::string WheelOfFortune::GetColorOld(){
  std::string color;
  frc::Color detectedColor = m_colorSensor.GetColor();

  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);

  if ( detectedColor.red > 0.4 && detectedColor.green < 0.4 && detectedColor.blue < 0.4) {
    color = "R";
  } else if ( detectedColor.red < 0.4 && detectedColor.green > 0.55 && detectedColor.blue < 0.4) {
    color = "G";
  } else if ( detectedColor.red < 0.4 && detectedColor.green < 0.5 && detectedColor.blue > 0.3) {
    color = "B";
  } else if ( detectedColor.red > 0.4 && detectedColor.green > 0.4 && detectedColor.blue < 0.3) {
    color = "Y";
  } else {
    //no color detected
    color = "X";
  }
  return color;
}

void WheelOfFortune::SpinToColor(){
  Current = GetColor();
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