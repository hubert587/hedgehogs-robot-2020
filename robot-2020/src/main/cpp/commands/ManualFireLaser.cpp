/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ManualFireLaser.h"

ManualFireLaser::ManualFireLaser() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ManualFireLaser::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualFireLaser::Execute() {}

// Called once the command ends or is interrupted.
void ManualFireLaser::End(bool interrupted) {}

// Returns true when the command should end.
bool ManualFireLaser::IsFinished() { return false; }
