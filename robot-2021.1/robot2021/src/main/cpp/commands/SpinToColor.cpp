/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SpinToColor.h"
#include "Robot.h"
#include "Constants.h"
#include <frc/Driverstation.h>

SpinToColor::SpinToColor(WheelOfFortune* SpinWheel): m_colorWheel{SpinWheel} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SpinToColor::Initialize() {
  m_colorWheel->DeployWheel();
  m_colorWheel->StartWheel();
}

// Called repeatedly when this Command is scheduled to run
void SpinToColor::Execute() {

} 

// Called once the command ends or is interrupted.
void SpinToColor::End(bool interrupted) {
  m_colorWheel->StopWheel();
  m_colorWheel->RetractWheel();
}

// Returns true when the command should end.
bool SpinToColor::IsFinished() { 
  //std::string Goal = frc::SmartDashboard::GetString("Color Defined", "R");
  std::string Goal = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  int CurrentColorValue = m_colorWheel->GetColor();
  std::string CurrentColor = m_colorWheel->ConvertColor(CurrentColorValue);
  if(Goal.length() > 0){
    if (Goal == "Y" && CurrentColor == "G") {
      return true;
    }
    if (Goal == "B" && CurrentColor == "R") {
      return true;
    }
    if (Goal == "G" && CurrentColor == "Y") {
      return true;
    }
    if (Goal == "R" && CurrentColor == "B") {
      return true;
    } 
  }
    return false;
}
