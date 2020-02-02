/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Intake.h"

Intake::Intake() {

}

void Intake::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Intake::IntakeSpeed(double speed) {
  m_InnerIntakeMotor.Set (speed);
  m_OuterIntakeMotor.Set (speed);

}

void Intake::ExtendIntake(bool extend) {
  ExtendIntakeSolenoid.Set(extend);
}

void Intake::StartIntake() {
  ExtendIntake(true);
  IntakeSpeed(0.5);
}

void Intake::StopIntake() {
  IntakeSpeed(0);
  ExtendIntake(false);
}