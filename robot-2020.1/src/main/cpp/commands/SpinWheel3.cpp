/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SpinWheel3.h"
#include "Robot.h"

SpinWheel3::SpinWheel3(WheelOfFortune* SpinWheel): m_colorWheel{SpinWheel} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SpinWheel3::Initialize() {
  //m_colorWheel->SpinWheel3Init();
  RedCount = 0;
  OnRed = false;
  //m_motor.Set(1);
  Done = false;
}

// Called repeatedly when this Command is scheduled to run
void SpinWheel3::Execute() {
  //m_colorWheel->SpinWheel3();
  int CurrentColorValue = m_colorWheel->GetColor();
  std::string CurrentColor = m_colorWheel->ConvertColor(CurrentColorValue);
  if(CurrentColor == "R") {
    RedCount++;
    OnRed = true;
  } else if(CurrentColor == "G" || CurrentColor == "Y" || CurrentColor == "B") {
    OnRed = false;
  }
}

// Called once the command ends or is interrupted.
void SpinWheel3::End(bool interrupted) {
  //Stop Motor
  //Retract Arm
}

// Returns true when the command should end.
bool SpinWheel3::IsFinished() { 
  return RedCount >= 7;
}
