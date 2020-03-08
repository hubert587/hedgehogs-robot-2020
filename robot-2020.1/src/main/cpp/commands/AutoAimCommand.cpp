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
        
    }

    
    
    void AutoAimCommand::Initialize() {
      
        m_startAngle = m_driveSubsystem->GetHeading();  // Set the value

        m_turnAngle = VisionGlobals::g_Angle;

        /*for (int i = 0; i < kLength; i++) {

            // Set the value
            m_ledBuffer[i].SetRGB(0,255,0);
        }
        m_led.SetLength(kLength);

        m_led.SetData(m_ledBuffer);
        m_led.Start();
*/


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
