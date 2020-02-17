/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <subsystems/Hopper.h>
#include <frc/smartdashboard/smartdashboard.h>

Hopper::Hopper() {
   m_HopperMotor.SetInverted(true);
   frc::SmartDashboard::PutNumber("Hopper Speed", 0.5);
}

// This method will be called once per scheduler run
void Hopper::Periodic() {}

void Hopper::HopperSpeed(double speed) {
  m_HopperMotor.Set(speed);

}

void Hopper::AutoHopper(){
  bool intake1 = IntakeBallDetector1.Get() == false;
  bool intake2 = IntakeBallDetector2.Get() == false;

  frc::SmartDashboard::PutNumber("Hopper intake1", intake1);
  frc::SmartDashboard::PutNumber("Hopper intake2", intake2);
  double speed = frc::SmartDashboard::GetNumber("Hopper Speed", 0.5);
  /*if (!intake1) {//&& !ShooterBallDetector.Get()){
    HopperSpeed(0);
  } else if (intake1) {
    HopperSpeed(1);
  } else {
    HopperSpeed(0);
  }*/
  if (intake1 || intake2) {//&& !ShooterBallDetector.Get()){
    HopperSpeed(speed);
  } else {
    HopperSpeed(0);
  }
}