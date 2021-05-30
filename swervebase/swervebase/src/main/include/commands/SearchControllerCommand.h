/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cmath>
#include <functional>
#include <initializer_list>
#include <memory>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/Trajectory.h>
#include <units/units.h>
#include <wpi/ArrayRef.h>

#include "frc2/command/CommandBase.h"
#include "frc2/command/CommandHelper.h"
#include "frc2/Timer.h"

#pragma once

namespace frc2 {

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
 * {@link Trajectory} with a swerve drive.
 *
 * <p>The command handles trajectory-following, Velocity PID calculations, and
 * feedforwards internally. This is intended to be a more-or-less "complete
 * solution" that can be used by teams without a great deal of controls
 * expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to
 * use the onboard PID functionality of a "smart" motor controller) may use the
 * secondary constructor that omits the PID and feedforward functionality,
 * returning only the raw module states from the position PID controllers.
 *
 * <p>The robot angle controller does not follow the angle given by
 * the trajectory but rather goes to the angle given in the final state of the
 * trajectory.
 */
template <size_t NumModules>
class SearchControllerCommand
    : public CommandHelper<CommandBase, SearchControllerCommand<NumModules>> {
  using voltsecondspermeter =
      units::compound_unit<units::voltage::volt, units::second,
                           units::inverse<units::meter>>;
  using voltsecondssquaredpermeter =
      units::compound_unit<units::voltage::volt, units::squared<units::second>,
                           units::inverse<units::meter>>;

 public:
  /**
   * Constructs a new SearchControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but
   * rather raw module states from the position controllers which need to be put
   * into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon
   * completion of the path- this is left to the user, since it is not
   * appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The rotation controller will calculate the rotation based on the
   * final pose in the trajectory, not the poses at each time step.
   *
   * @param trajectory      The trajectory to follow.
   * @param pose            A function that supplies the robot pose,
   *                        provided by the odometry class.
   * @param kinematics      The kinematics for the robot drivetrain.
   * @param xController     The Trajectory Tracker PID controller
   *                        for the robot's x position.
   * @param yController     The Trajectory Tracker PID controller
   *                        for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller
   *                        for angle for the robot.
   * @param output          The raw output module states from the
   *                        position controllers.
   * @param requirements    The subsystems to require.
   */
  SearchControllerCommand(
      frc::Trajectory trajectoryA,
      frc::Trajectory trajectoryB, 
      std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)>
          output,
      std::initializer_list<Subsystem*> requirements): 
      m_trajectory(trajectoryA),
      m_trajectoryB(trajectoryB),
      m_pose(pose),
      m_kinematics(kinematics),
      m_xController(std::make_unique<frc2::PIDController>(xController)),
      m_yController(std::make_unique<frc2::PIDController>(yController)),
      m_thetaController(
          std::make_unique<frc::ProfiledPIDController<units::radians>>(
              thetaController)),
      m_outputStates(output) {
  this->AddRequirements(requirements);
};

  /**
   * Constructs a new SearchControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but
   * rather raw module states from the position controllers which need to be put
   * into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon
   * completion of the path- this is left to the user, since it is not
   * appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The rotation controller will calculate the rotation based on the
   * final pose in the trajectory, not the poses at each time step.
   *
   * @param trajectory      The trajectory to follow.
   * @param pose            A function that supplies the robot pose,
   *                        provided by the odometry class.
   * @param kinematics      The kinematics for the robot drivetrain.
   * @param xController     The Trajectory Tracker PID controller
   *                        for the robot's x position.
   * @param yController     The Trajectory Tracker PID controller
   *                        for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller
   *                        for angle for the robot.
   * @param output          The raw output module states from the
   *                        position controllers.
   * @param requirements    The subsystems to require.
   */
  SearchControllerCommand(
      frc::Trajectory trajectoryA,
      frc::Trajectory trajectoryB, 
      std::function<frc::Pose2d()> pose,
      frc::SwerveDriveKinematics<NumModules> kinematics,
      frc2::PIDController xController, frc2::PIDController yController,
      frc::ProfiledPIDController<units::radians> thetaController,
      std::function<void(std::array<frc::SwerveModuleState, NumModules>)>
          output,
      wpi::ArrayRef<Subsystem*> requirements = {}) : 
      m_trajectory(trajectoryA),
      m_trajectoryB(trajectoryB),
      m_pose(pose),
      m_kinematics(kinematics),
      m_xController(std::make_unique<frc2::PIDController>(xController)),
      m_yController(std::make_unique<frc2::PIDController>(yController)),
      m_thetaController(
          std::make_unique<frc::ProfiledPIDController<units::radians>>(
              thetaController)),
      m_outputStates(output) {
  this->AddRequirements(requirements);
}



  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  static int Path;
  frc::Trajectory m_trajectory;
  frc::Trajectory m_trajectoryB;
  std::function<frc::Pose2d()> m_pose;
  frc::SwerveDriveKinematics<NumModules> m_kinematics;
  std::unique_ptr<frc2::PIDController> m_xController;
  std::unique_ptr<frc2::PIDController> m_yController;
  std::unique_ptr<frc::ProfiledPIDController<units::radians>> m_thetaController;
  std::function<void(std::array<frc::SwerveModuleState, NumModules>)>
      m_outputStates;

  frc2::Timer m_timer;
  units::second_t m_prevTime;
  frc::Pose2d m_finalPose;
};
}  // namespace frc2

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <memory>
#include "Robot.h"

namespace frc2 {

template <size_t NumModules>
int SearchControllerCommand<NumModules>::Path = 0;


template <size_t NumModules>
void SearchControllerCommand<NumModules>::Initialize() {
if(Path == 0){
  frc::DigitalInput Ultrasonic{3};
  bool tmp = Ultrasonic.Get();
  if (tmp) Path = 1;
  else Path = 2;
  std::cout<<"Determine Path: "<<Path<<" tmp " << tmp << "\n";
}
std::cout<<"Path: "<<Path<<"\n";
 if(Path == 2){
     m_trajectory = m_trajectoryB;
 }
  m_finalPose = m_trajectory.Sample(m_trajectory.TotalTime()).pose;

  m_timer.Reset();
  m_timer.Start();
}

template <size_t NumModules>
void SearchControllerCommand<NumModules>::Execute() {
  auto curTime = units::second_t(m_timer.Get());

  auto m_desiredState = m_trajectory.Sample(curTime);
  auto m_desiredPose = m_desiredState.pose;

  auto m_poseError = m_desiredPose.RelativeTo(m_pose());

  auto targetXVel = units::meters_per_second_t(m_xController->Calculate(
      (m_pose().Translation().X().template to<double>()),
      (m_desiredPose.Translation().X().template to<double>())));
  auto targetYVel = units::meters_per_second_t(m_yController->Calculate(
      (m_pose().Translation().Y().template to<double>()),
      (m_desiredPose.Translation().Y().template to<double>())));

  // Profiled PID Controller only takes meters as setpoint and measurement
  // The robot will go to the desired rotation of the final pose in the
  // trajectory, not following the poses at individual states.
  auto targetAngularVel =
      units::radians_per_second_t(m_thetaController->Calculate(
          m_pose().Rotation().Radians(), m_finalPose.Rotation().Radians()));

  auto vRef = m_desiredState.velocity;

  targetXVel += vRef * m_poseError.Rotation().Cos();
  targetYVel += vRef * m_poseError.Rotation().Sin();

  auto targetChassisSpeeds =
      frc::ChassisSpeeds{targetXVel, targetYVel, targetAngularVel};

  auto targetModuleStates =
      m_kinematics.ToSwerveModuleStates(targetChassisSpeeds);

  m_outputStates(targetModuleStates);
}

template <size_t NumModules>
void SearchControllerCommand<NumModules>::End(bool interrupted) {
  m_timer.Stop();
}

template <size_t NumModules>
bool SearchControllerCommand<NumModules>::IsFinished() {
  return m_timer.HasElapsed(m_trajectory.TotalTime());
}

}  // namespace frc2
