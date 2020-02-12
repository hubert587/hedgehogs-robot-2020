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
  m_InnerIntakeMotor.Set(speed);
  m_OuterIntakeMotor.Set(speed);
}

void Intake::AdjustIntake(int adjustIntake) {
  if (adjustIntake == 0) {
    m_LeftInnerIntakeSolenoid.Set(false);
    m_LeftOuterIntakeSolenoid.Set(false);
    m_RightInnerIntakeSolenoid.Set(false);
    m_RightOuterIntakeSolenoid.Set(false);
    intakeTracker = 0;
  } else if (adjustIntake == 1) {
    m_LeftInnerIntakeSolenoid.Set(true);
    m_LeftOuterIntakeSolenoid.Set(false);
    m_RightInnerIntakeSolenoid.Set(true);
    m_RightOuterIntakeSolenoid.Set(false);
    intakeTracker = 1;
  } else if (adjustIntake == 2) {
    m_LeftInnerIntakeSolenoid.Set(true);
    m_LeftOuterIntakeSolenoid.Set(true);
    m_RightInnerIntakeSolenoid.Set(true);
    m_RightOuterIntakeSolenoid.Set(true);
    intakeTracker = 2;
  }
}

void Intake::ShootingIntakePositioning() {
  if (intakeTracker == 2) {
    AdjustIntake(1);
  }
}

void Intake::StartIntake() {
  AdjustIntake(2);
  IntakeSpeed(0.5);
}

void Intake::StopIntake() {
  IntakeSpeed(0);
  AdjustIntake(0);
}