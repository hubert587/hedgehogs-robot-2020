/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoAimCommand.h"

AutoAimCommand::AutoAimCommand(DriveSubsystem* driveSubsystem)
    : m_driveSubsystem{driveSubsystem} {

        m_turnRate = 0.5;

    
    }

    
    
    void AutoAimCommand::Initialize() {

        m_startAngle = m_driveSubsystem->GetHeading();

    }
    
    void AutoAimCommand::Execute() {  
        
        m_driveSubsystem->Drive(units::meters_per_second_t(0),
                      units::meters_per_second_t(0),
                      units::radians_per_second_t(m_turnRate),
                      false);
                      
    }

    void AutoAimCommand::End(bool interrupted) {}

    bool AutoAimCommand::IsFinished() {

        return (m_driveSubsystem->GetHeading() > m_startAngle + m_turnAngle);
                
    }
