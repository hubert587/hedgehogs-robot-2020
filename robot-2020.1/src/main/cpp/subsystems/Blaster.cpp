/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Blaster.h"
#include <frc/shuffleboard/Shuffleboard.h>

Blaster::Blaster() {
    m_LeftBlasterWheel.SetInverted(true);
    frc::SmartDashboard::PutNumber("Left Blaster", 0.5);
    frc::SmartDashboard::PutNumber("Right Blaster", 0.5);
}

void Blaster::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Blaster::BlasterSpeed(double leftSpeed, double rightSpeed) {
  if(leftSpeed > 0) {
    leftSpeed = frc::SmartDashboard::GetNumber("Left Blaster", 1);
    rightSpeed = frc::SmartDashboard::GetNumber("Right Blaster", 1);
  }
  m_LeftBlasterWheel.Set(leftSpeed);
  m_RightBlasterWheel.Set(rightSpeed);
}

void Blaster::AngleChange(bool angledUp) {
  if (angledUp == true) {
    m_AngleSolenoid.Set(frc::DoubleSolenoid::kForward);
  } else {
    m_AngleSolenoid.Set(frc::DoubleSolenoid::kReverse);
  }
  
}

//add angle adjustment