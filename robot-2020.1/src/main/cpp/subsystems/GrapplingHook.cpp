/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/GrapplingHook.h"

GrapplingHook::GrapplingHook() {

}

void GrapplingHook::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void GrapplingHook::GrapplingHookSpeed(double speed) {
  m_ClimberLeftMotor.Set (speed);
  m_ClimberRightMotor.Set (speed);

}

void GrapplingHook::GrapplingHookAdjustmentSpeed(double speed) {
  m_ClimberAdjustmentMotor.Set (speed);

}

void GrapplingHook::Deploy(bool deploy) {
  m_ClimbSolenoid.Set(deploy);
}
