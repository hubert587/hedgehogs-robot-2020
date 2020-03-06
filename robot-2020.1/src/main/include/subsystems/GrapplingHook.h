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
#include <frc/Solenoid.h> 
#include <frc/Joystick.h>


class GrapplingHook : public frc2::SubsystemBase {

 public:
  GrapplingHook();
  void GrapplingHookSpeed(double speed);

  void GrapplingHookAdjustmentSpeed(double speed);

  void Execute();

  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Deploy(bool deploy);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  bool EndGameStarted;

  WPI_TalonSRX m_ClimberLeftMotor{ canIDs::kClimberLeftMptorCanID };
  WPI_TalonSRX m_ClimberRightMotor{ canIDs::kClimberRightMotorCanID };
  WPI_TalonSRX m_ClimberAdjustmentMotor{ canIDs::kClimberAdjustmentMotorCanID };
  frc::Solenoid m_ClimbSolenoid{solenoidIDs::kClimbSolenoid};
   frc::Joystick m_codriverController{OIConstants::CoDriver};

};
