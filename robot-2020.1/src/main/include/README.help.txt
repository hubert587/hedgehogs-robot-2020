Open program by going to desktop and starting the icon with title "FRC VS Code 2020" 
If you need to open the code. 
Select File->open folder...
C:\code\hedgehogs-robot-2020\hedgehogs-robot-2020 and select robot-2020.1

The code for the color wheel prototype was done in:
C:\Users\robodog\Documents\2020 examples and select motorcontroller

Some more important setup things we need to do:
In RobotContainer.h we need to create instances of all our subsystems
  You can use DriveSubsystem as a example.
  #include "subsystems/DriveSubsystem.h"
  DriveSubsystem m_drive;
 
Need to add a hopper subsystem

Need to create the instances of the commands that were created in the command folders.
These can be added into RobotContainer.h also, something like 
  #include "commands/AutoFireLaser.h"
  AutoFireLaser m_autoFireLaserCommand;
The commands can than be bound to buttons on the joystick/controllers. See below.

Can start adding behavior to the subsystems.
The intake would be an easy one to start with. 
Will need to add a talon instance to the header file. See instructions below.
Need to add a instance of a solendiod to the header file.
Add functions/methods to 
   extend intake, ie enable the solendiod.
   retract intake, ie disable the solendiod.
   control for the motor    


To add a talon motor controller
   add the include (.h) file so the program knows what a talon is
   #include <ctre/Phoenix.h>
   define the instance in the header file of the motor controller.
   WPI_TalonSRX m_leftMotor{1};
   change m_leftMotor to a name appropriate for the use.
   Change the 1 above to the CAN bus ID we plan to use for the controller. 
   Each one has to be a different number.
   Best practice is to define a constant in the Constants.h file under include folder
   In the code file (.cpp) add the control of the motor controller.
   m_leftMotor.Set(val); where val is between -1.0 and 1.0

To add a sparkmax motor controller see steps for talon
   #include <rev/CANSparkMax.h>
   rev::CANSparkMax m_driveMotor{1};

To add a solenoid to control a pneumatic cylinder it is similar to a motor. 
  The number/s are based on the position on the PCM controller they are 
  connected to and are 1-8. A regular solenoid just takes one and for the 
  control is either powered or not so takes a true or false for the Set.  
  #include <frc/Solenoid.h> 
  frc::Solenoid BeakOpenSolenoid{4};
  BeakOpenSolenoid.Set(true);
  BeakOpenSolenoid.Set(false);

  A double solenoid uses two numbers for ports on the PCM controller. For the control
  it takes a value telling if it is in (kReverse) or out (kForward).
  #include <frc/DoubleSolenoid.h>
  frc::DoubleSolenoid m_hatchSolenoid{1,2};
  m_hatchSolenoid.Set(frc::DoubleSolenoid::kForward);
  m_hatchSolenoid.Set(frc::DoubleSolenoid::kReverse);
 
 
To add a button press type of command, simple command like triggering a solenoid.
For more complex commands define a command class under the commands folders and skip the
the next couple of lines. 
  In RobotContainer.h define the command. 
  m_hatch is the subsystem you want the command to work on.
  GrabHatch() is the function/method which is called on the subsystem.
  m_grabHatch is the name of the command that is created.
  frc2::InstantCommand m_grabHatch{[this] { m_hatch.GrabHatch(); }, {&m_hatch}};
  
  In RobotContainer.cpp inside of the function/method ConfigureButtonBindings()
  This is where you define a mapping between a button and what commands is called.
  The command can either be from one define as above or defined under the command folder.
  
  Here are some examples, the first parameter for JoystickButton is the usb controller/joystick
  which you want to use the execute the command. It is defined in RobotContainer.h
  We probably have to define a controller istance for the co-driver.
  The second parameter is the number of the button to use to run the command. If you pull up
  the driver station and select the usb tab it will show which slot the button relates to.
  You will have to count to figure out the proper number. I think they start at 1.
  The command instance (&m_grabHatch) is passed into the WhenPressed or WhenReleased. 
  
  // Grab the hatch when the 'A' button is pressed.
  frc2::JoystickButton(&m_driverController, 1).WhenPressed(&m_grabHatch);
  // Release the hatch when the 'B' button is pressed.
  frc2::JoystickButton(&m_driverController, 2).WhenPressed(&m_releaseHatch);
  // While holding the shoulder button, drive at half speed
  frc2::JoystickButton(&m_driverController, 6)
      .WhenPressed(&m_driveHalfSpeed)
      .WhenReleased(&m_driveFullSpeed);

  In the subsystem implement the function/method called by the command
  so for m_hatch.GrabHatch() you would add to the hatch subsystem the following procedure/method
  in the header file (.h)
  void GrabHatch();
  in the code file (.cpp)
  void hatchSubsystem::GrabHatch() {
     // code to do something here.
  }
  
  
  
  library links
  https://www.kauailabs.com/dist/frc/2020/navx_frc.json
  http://revrobotics.com/content/sw/color-sensor-v3/sdk/REVColorSensorV3.json
  https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json
  http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json