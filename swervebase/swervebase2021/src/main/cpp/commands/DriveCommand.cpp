/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <units/units.h>
#include "commands/DriveCommand.h"
#include "Constants.h"

DriveCommand::DriveCommand(DriveSubsystem* driveSubsystem, frc::Joystick* joystick)
    : m_driveSubsystem{driveSubsystem},
    m_joystick{joystick} {}
    //m_xboxController{xboxController} {}
    
    
    void DriveCommand::Initialize() {}
    
    void DriveCommand::Execute() {


        int targetYes = m_joystick->GetRawButton(1);
        //int targetYes = m_xboxController->GetAButton();

        if(targetYes) {

            


        } else {
        
        /*m_driveSubsystem->Drive(units::meters_per_second_t(
                          m_xboxController->GetY(frc::GenericHID::kLeftHand)),
                      units::meters_per_second_t(
                          m_xboxController->GetY(frc::GenericHID::kRightHand)),
                      units::radians_per_second_t(
                          m_xboxController->GetX(frc::GenericHID::kLeftHand)),
                        
                      false);*/
        
        m_driveSubsystem->Drive(
            
            
            
                units::meters_per_second_t(
                          m_joystick->GetRawAxis(0)),
                      units::meters_per_second_t(
                          m_joystick->GetRawAxis(1)),
                      units::radians_per_second_t(
                          m_joystick->GetRawAxis(2)),
                        
                      false);
      
        
        }
    }

    void DriveCommand::End(bool interrupted) {



    }

    bool DriveCommand::IsFinished() {
        
        return false;
    
    }
