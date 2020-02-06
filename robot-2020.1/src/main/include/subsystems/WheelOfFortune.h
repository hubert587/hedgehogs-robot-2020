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
#include <iostream>
#include <string>

class WheelOfFortune : public frc2::SubsystemBase {

 public:
  WheelOfFortune();

    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  std::string GetColorOld();
  int GetColor();
  void SpinToColorInit();
  void SpinToColor();
  void SpinWheel3Init();
  void SpinWheel3();
  bool IsDone();
  void StartWheel();
  void StopWheel();
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //spinner

  //color detector
  bool done = false;
  int RedCount = 0;
  bool onRed = false;
  std::string Goal = "R";
  std::string Current = "R";
  WPI_TalonSRX m_motor{kColorWheelCanID};
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
};
