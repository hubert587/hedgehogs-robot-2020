/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/GrapplingHook.h"

GrapplingHook::GrapplingHook() {

  EndGameStarted = false;

}

void GrapplingHook::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void GrapplingHook::GrapplingHookSpeed(double speed) {
  m_ClimberLeftMotor.Set (speed);
  m_ClimberRightMotor.Set (-speed);
}

void GrapplingHook::GrapplingHookAdjustmentSpeed(double speed) {
  m_ClimberAdjustmentMotor.Set (speed);
}

void GrapplingHook::Execute(){

if (EndGameStarted == true){

  double speed = m_codriverController.GetRawAxis(1);
  GrapplingHookSpeed(speed);
  
}

}

void GrapplingHook::Deploy(bool deploy) {
  m_ClimbSolenoid.Set(deploy);
  EndGameStarted = true;
}
