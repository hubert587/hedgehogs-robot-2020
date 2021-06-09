/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <Constants.h> 
#include <frc/DoubleSolenoid.h> 

class Blaster : public frc2::SubsystemBase{

 public:
  Blaster();
  void BlasterSpeed(double leftSpeed, double rightSpeed);
  void AngleChange(bool angledUp);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //fly wheels
  WPI_TalonSRX m_LeftBlasterWheel{ canIDs::kLeftBlasterWheelCanID };
  WPI_TalonSRX m_RightBlasterWheel{ canIDs::kRightBlasterWheelCanID };
  frc::DoubleSolenoid m_AngleSolenoid{solenoidIDs::kAngleSolenoid1, solenoidIDs::kAngleSolenoid2};

  

};
