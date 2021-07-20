/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <subsystems/WheelOfFortune.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>

WheelOfFortune::WheelOfFortune() {

}

void WheelOfFortune::Periodic() {
  // Implementation of subsystem periodic method goes here.
  
}

int WheelOfFortune::GetColor() {

  frc::Color detectedColor = m_colorSensor.GetColor();
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  double tolerance = frc::SmartDashboard::GetNumber("Tolerance", .8);
  rev::ColorMatch Matcher;
  for (int x = 0; x < 4; x++) {
    Matcher.AddColorMatch(kColorCodes[x]);
  }
  Matcher.SetConfidenceThreshold(tolerance);
  std::optional<frc::Color> matchedColor = Matcher.MatchColor(detectedColor);
  frc::SmartDashboard::PutNumber("Matched Red", matchedColor.has_value() ? matchedColor.value().red : 0);
  frc::SmartDashboard::PutNumber("Matched Green", matchedColor.has_value() ? matchedColor.value().green : 0);
  frc::SmartDashboard::PutNumber("Matched Blue", matchedColor.has_value() ? matchedColor.value().blue : 0);
  int colorIndex = -1;
  for (int color = 0; color < 4; color++) {
    if (matchedColor == kColorCodes[color]) {
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

void WheelOfFortune::StartWheel() {
  m_motor.Set(1);
}

void WheelOfFortune::StopWheel() {
  m_motor.Set(0);
}

void WheelOfFortune::DeployWheel() {
  m_colorSolenoid.Set(true);
}

void WheelOfFortune::RetractWheel() {
  m_colorSolenoid.Set(false);
}

void WheelOfFortune::CancelWheel() {
  StopWheel();
  RetractWheel();
}