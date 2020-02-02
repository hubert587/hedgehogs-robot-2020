/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Blaster.h"

Blaster::Blaster() {

}

void Blaster::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Blaster::BlasterSpeed(double leftSpeed, double rightSpeed) {
  m_LeftBlasterWheel.Set(leftSpeed);
  m_RightBlasterWheel.Set(rightSpeed);
}