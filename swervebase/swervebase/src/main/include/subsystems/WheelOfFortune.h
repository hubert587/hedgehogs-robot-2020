/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <ctre/Phoenix.h>
#include <rev/ColorSensorV3.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/Solenoid.h> 
#include <iostream>
#include <string>

class WheelOfFortune : public frc2::SubsystemBase {

 public:
  WheelOfFortune();

  frc::Color kColorCodes[4] = {
      frc::Color(.42, .50, .08),
      frc::Color(.61, .32, .07),
      frc::Color(.24, .57, .19),
      frc::Color(.19, .38, .34),
  };
  double kColorTolerance = .8; 
    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  int GetColor();
  std::string ConvertColor(int colorIndex);
  void StartWheel();
  void StopWheel();
  void DeployWheel();
  void RetractWheel();
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //spinner

  //color detector
  WPI_TalonSRX m_motor { kColorWheelCanID };
  rev::ColorSensorV3 m_colorSensor { frc::I2C::Port::kOnboard };
  frc::Solenoid m_colorSolenoid { solenoidIDs::kColorSolenoid };
};
