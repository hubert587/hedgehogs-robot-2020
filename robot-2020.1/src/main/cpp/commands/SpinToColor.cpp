/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SpinToColor.h"
#include "Robot.h"

SpinToColor::SpinToColor(WheelOfFortune* SpinWheel): m_colorWheel{SpinWheel} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SpinToColor::Initialize() {
  m_colorWheel->SpinToColorInit();
}

// Called repeatedly when this Command is scheduled to run
void SpinToColor::Execute() {
  m_colorWheel->SpinToColor();
} 

// Called once the command ends or is interrupted.
void SpinToColor::End(bool interrupted) {
//Retract Arm
}

// Returns true when the command should end.
bool SpinToColor::IsFinished() { 
  return m_colorWheel->IsDone(); 
}
