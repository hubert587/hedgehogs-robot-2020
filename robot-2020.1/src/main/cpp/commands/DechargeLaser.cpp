/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DechargeLaser.h"

DechargeLaser::DechargeLaser(Blaster* blaster): m_blaster{blaster}  {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void DechargeLaser::Initialize() {
  m_blaster->BlasterSpeed(0);
}

// Called repeatedly when this Command is scheduled to run
void DechargeLaser::Execute() {}

// Called once the command ends or is interrupted.
void DechargeLaser::End(bool interrupted) {}

// Returns true when the command should end.
bool DechargeLaser::IsFinished() { return true; }
