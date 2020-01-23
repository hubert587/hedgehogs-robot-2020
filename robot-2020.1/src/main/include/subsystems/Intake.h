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

class Intake : public frc2::SubsystemBase {

 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //deployer
  //Need to figure out how to do the pneumatics for this darn thingy thing

  //intaker
  WPI_TalonSRX m_OuterIntakeMotor{ canIDs::kOuterIntakeMotorCanID };
  WPI_TalonSRX m_InnerIntakeMotor{ canIDs::kInnerIntakeMotorCanID };
};
