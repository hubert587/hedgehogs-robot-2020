/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/GrapplingHook.h"

GrapplingHook::GrapplingHook() {

  EndGameStarted = false;
  m_ClimbSolenoid.Set(false);
}

void GrapplingHook::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void GrapplingHook::GrapplingHookSpeed(double speed) {
  m_ClimberLeftMotor.Set (-speed);
  m_ClimberRightMotor.Set (-speed);
}

void GrapplingHook::GrapplingHookAdjustmentSpeed(double speed) {
  m_ClimberAdjustmentMotor.Set (speed);
}

void GrapplingHook::Execute(){

if (EndGameStarted == true){

  double speed = m_codriverController.GetRawAxis(1);
  speed = speed * 0.7;
  if (fabs(speed) < 0.1) m_ClimbSolenoid.Set(false);
  else if (speed < 0) {
    m_ClimbSolenoid.Set(true);
    speed = speed * 1.1;
  }
  else m_ClimbSolenoid.Set(false);
  GrapplingHookSpeed(speed);
  
}

}

void GrapplingHook::Deploy(bool deploy) {
  m_ClimbSolenoid.Set(deploy);
  if (EndGameStarted) { // skip this for the first loop to let the soleniod engage
      GrapplingHookSpeed(-1.0);
  }
  
  EndGameStarted = true;

}
