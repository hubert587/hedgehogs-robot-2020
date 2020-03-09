/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoAimCommand.h"
#include <Globals.h>



AutoAimCommand::AutoAimCommand(DriveSubsystem* driveSubsystem)
    : m_driveSubsystem{driveSubsystem} {

        m_turnRate = 0.5;
        frc::SmartDashboard::PutString("Trace - Aim", "construct");
        
    }

    
    
    void AutoAimCommand::Initialize() {
      
        frc::SmartDashboard::PutString("Trace - Aim", "init");
        m_startAngle = m_driveSubsystem->GetHeading();  // Set the value
        m_turnAngle = VisionGlobals::g_Angle;
        frc::SmartDashboard::PutNumber("AutoAimInitStart", m_startAngle);
        frc::SmartDashboard::PutNumber("AutoAimInitTurn", m_turnAngle);
        frc::SmartDashboard::PutNumber("AutoAimInitAngle", VisionGlobals::g_Angle);

    }
    
    void AutoAimCommand::Execute() {  
        frc::SmartDashboard::PutString("Trace - Aim", "exe");        
        m_driveSubsystem->Drive(units::meters_per_second_t(0),
                      units::meters_per_second_t(0),
                      units::radians_per_second_t(m_turnRate),
                      false);
   
        frc::SmartDashboard::PutNumber("AutoAimInitStart", m_startAngle);
        frc::SmartDashboard::PutNumber("AutoAimInitTurn", m_turnAngle);
        frc::SmartDashboard::PutNumber("AutoAimInitAngle", VisionGlobals::g_Angle);
    }

    void AutoAimCommand::End(bool interrupted) {}

    bool AutoAimCommand::IsFinished() {
        frc::SmartDashboard::PutNumber("AutoAimInitFinish", abs(m_driveSubsystem->GetHeading() - (m_startAngle + m_turnAngle)));
        return (abs(m_driveSubsystem->GetHeading() - (m_startAngle + m_turnAngle)) < .01);
                
    }
