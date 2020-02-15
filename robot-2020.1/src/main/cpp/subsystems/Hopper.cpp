/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <subsystems/Hopper.h>


Hopper::Hopper() {}

// This method will be called once per scheduler run
void Hopper::Periodic() {}

void Hopper::HopperSpeed(double speed) {
  m_HopperMotor.Set (speed);

}

void Hopper::AutoHopper(){
  if (!IntakeBallDetector.Get() && !ShooterBallDetector.Get()){
    HopperSpeed(0);
  } else if (!IntakeBallDetector.Get()) {
    HopperSpeed(1);
  } else {
    HopperSpeed(0);
  }

}