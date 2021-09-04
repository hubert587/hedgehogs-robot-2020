/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/GrapplingHook.h"

GrapplingHook::GrapplingHook() {

  EndGameStarted = false;
  toggle = true;
  m_ClimbSolenoid.Set(false);
  frc::SmartDashboard::PutBoolean("Rachet", m_ClimbSolenoid.Get());
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
  static int count = 0;
  if (EndGameStarted == true){

    double speed = m_codriverController.GetRawAxis(1);
  
    if (fabs(speed) < 0.1) {
      speed = 0;
      //m_ClimbSolenoid.Set(false);
      count = 0;
      GrapplingHookSpeed(speed); 
    }
    else if (speed < 0) {
      //m_ClimbSolenoid.Set(true);
      count++;
      if(count>3){
         GrapplingHookSpeed(speed); 
      }else{
        GrapplingHookSpeed(0); 
      }
    }
    else {
      speed = speed * 0.7;
      //m_ClimbSolenoid.Set(false);
      count = 0;
      GrapplingHookSpeed(speed); 
    }

  }
  
}

void GrapplingHook::Deploy(bool deploy) {

  m_ClimbSolenoid.Set(toggle);

  if (deploy)
     toggle = ! toggle;

  if (EndGameStarted) { // skip this for the first loop to let the soleniod engage
     // GrapplingHookSpeed(-1.0);
  }
  
  EndGameStarted = true;
  frc::SmartDashboard::PutBoolean("Rachet", m_ClimbSolenoid.Get());
}

void GrapplingHook::Solenoid(bool deploy) {
  m_ClimbSolenoid.Set(deploy);
  frc::SmartDashboard::PutBoolean("Rachet", m_ClimbSolenoid.Get());
}