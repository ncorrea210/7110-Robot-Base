// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "RobotContainer.h"

#include <utility>
#include <cmath>
#include <numbers>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/ConditionalCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/Limelight.h"
#include "commands/autos/TestSeqCMD.h"

using namespace DriveConstants;

static bool cone = true;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // m_MainEncoder.SetDutyCycleRange(1/1025, 1024/1025);
  //   frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftStick).WhenPressed(
    // frc2::RunCommand([this] {m_drive.ResetGyro();}, {&m_drive})).WhenReleased(DefaultDriveCMD(&m_drive, 
    //       [this] {return (-m_speedLimitx.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY() < 0 ? -(m_driverController.GetLeftY() * m_driverController.GetLeftY()) : (m_driverController.GetLeftY() * m_driverController.GetLeftY()), 0.05)) * (double)AutoConstants::kMaxSpeed);}, 
    //       [this] {return (m_speedLimity.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX() < 0 ? -(m_driverController.GetLeftX() * m_driverController.GetLeftX()) : (m_driverController.GetLeftX() * m_driverController.GetLeftX()), 0.05)) * (double)AutoConstants::kMaxSpeed);},
    //       [this] {return (m_speedLimitz.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), 0.1)) * (double)AutoConstants::kMaxAngularSpeed);}, 
    //       [this] {return true;}));

  m_chooser.AddOption("Test", &TestSeq);
  m_chooser.AddOption("Balance Path", &BalanceSeq);
  m_chooser.AddOption("Score Mid And Leave", &PlaceMidNLeave);

  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);

  // Configure the button bindings
  ConfigureButtonBindings();

  // printf("x: %5.2f y: %5.2f rot: %5.2f\n", xSpeed, ySpeed, rot);

    m_drive.SetDefaultCommand(DefaultDriveCMD(&m_drive, 
          [this] {return (-m_speedLimitx.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY() < 0 ? -(m_driverController.GetLeftY() * m_driverController.GetLeftY()) : (m_driverController.GetLeftY() * m_driverController.GetLeftY()), 0.025)) * (double)DriveConstants::kMaxSpeed);}, 
          [this] {return (m_speedLimity.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX() < 0 ? -(m_driverController.GetLeftX() * m_driverController.GetLeftX()) : (m_driverController.GetLeftX() * m_driverController.GetLeftX()), 0.025)) * (double)DriveConstants::kMaxSpeed);},
          [this] {return (m_speedLimitz.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), 0.05)) * (double)DriveConstants::kMaxAngularSpeed);}, 
          [this] {return true;}));


  frc2::RunCommand([] {frc::SmartDashboard::PutNumber("X Offset", hb::limeLight::GetX());});
  frc2::RunCommand([] {frc::SmartDashboard::PutNumber("Y Offset", hb::limeLight::GetY());});
  frc2::RunCommand([this] {frc::SmartDashboard::PutBoolean("Cone/Cube", m_last);});
}

void RobotContainer::ConfigureButtonBindings() {

  
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA).WhenPressed(
  //   frc2::RunCommand([this] { m_extension.RunExtension(-0.5);}, {&m_extension})).WhenReleased(
  //     frc2::RunCommand([this] { m_extension.RunExtension(0);}, {&m_extension}));
  
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY).WhenPressed(
  //   frc2::RunCommand([this] { m_extension.RunExtension(0.5);}, {&m_extension})).WhenReleased(
  //     frc2::RunCommand([this] { m_extension.RunExtension(0);}, {&m_extension}));

  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).WhenPressed(
  //   frc2::RunCommand([this] {m_actuator.Run(0.75);}, {&m_actuator})).WhenReleased(
  //     frc2::RunCommand([this] {m_actuator.Run(0);}, {&m_actuator}));

  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB).WhenPressed(
  //   frc2::RunCommand([this] {m_actuator.Run(-0.75);}, {&m_actuator})).WhenReleased(
  //     frc2::RunCommand([this] {m_actuator.Run(0);}, {&m_actuator}));

  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kLeftBumper).WhenPressed(
    frc2::RunCommand([this] {m_clamp.RunClamp(-0.25);}, {&m_clamp})).WhenReleased(
      frc2::RunCommand([this] {m_clamp.RunClamp(0);}, {&m_clamp}));

  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kRightBumper).WhenPressed(
    frc2::RunCommand([this] {m_clamp.RunClamp(0.25);}, {&m_clamp})).WhenReleased(
      frc2::RunCommand([this] {m_clamp.RunClamp(0);}, {&m_clamp}));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper).WhenPressed(
    frc2::InstantCommand([this] {m_drive.ZeroHeading();}, {&m_drive}));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhenPressed(
    frc2::InstantCommand([this] {hb::limeLight::SetLED(hb::limeLight::LEDMode::kOn);}));
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart).WhenPressed(
    frc2::InstantCommand([this] {hb::limeLight::SetLED(hb::limeLight::LEDMode::kOff);}));

  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftStick).WhenPressed(
  //   frc2::RunCommand([this] {m_drive.Drive(0_mps, 0_mps, units::radian_t(-std::numbers::pi));}, {&m_drive})).WhenReleased(DefaultDriveCMD(&m_drive, 
  //         [this] {return (-m_speedLimitx.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY() < 0 ? -(m_driverController.GetLeftY() * m_driverController.GetLeftY()) : (m_driverController.GetLeftY() * m_driverController.GetLeftY()), 0.05)) * (double)DriveConstants::kMaxSpeed);}, 
  //         [this] {return (m_speedLimity.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX() < 0 ? -(m_driverController.GetLeftX() * m_driverController.GetLeftX()) : (m_driverController.GetLeftX() * m_driverController.GetLeftX()), 0.05)) * (double)DriveConstants::kMaxSpeed);},
  //         [this] {return (m_speedLimitz.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), 0.1)) * (double)DriveConstants::kMaxAngularSpeed);}, 
  //         [this] {return true;}));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightStick).WhenPressed(LLTarget);
  
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY).WhenPressed(InFrameCMD(&m_extension, &m_actuator));
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA).WhenPressed(PlaceMidConeCMD(&m_extension, &m_actuator));
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB).WhenPressed(PickUpCMD(&m_extension, &m_actuator, cone));
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).WhenPressed(CubeGrapPosCMD(&m_extension, &m_actuator));
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).WhenPressed(
    frc2::InstantCommand([this] {cone ? cone = false : cone = true;}));
  

}


frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}


// frc2::Command* RobotContainer::GetAutonomousCommand() {
//   // Set up config for trajectory
//   frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
//                                AutoConstants::kMaxAcceleration);
//   // Add kinematics to ensure max speed is actually obeyed
//   config.SetKinematics(m_drive.kDriveKinematics);

//   auto ForwardTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
//     frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
//     {frc::Translation2d(-2.5_m, 0_m), frc::Translation2d(-2.6_m, 0_m)},
//     frc::Pose2d(-3_m, 0_m, frc::Rotation2d(0_deg)), config);

//     frc::ProfiledPIDController<units::radians> thetaController{
//       AutoConstants::kPThetaController, 0, 0,
//       AutoConstants::kThetaControllerConstraints};

//   thetaController.EnableContinuousInput(units::radian_t(-std::numbers::pi),
//                                         units::radian_t(std::numbers::pi));

//   m_Routine = frc::SmartDashboard::GetNumber("Auto", 0);

//   frc::Trajectory m_SelectedTrajectory = ForwardTrajectory;

//   frc2::SwerveControllerCommand<4> swerveControllerCommand(
//       m_SelectedTrajectory, [this]() { return m_drive.GetPose(); },

//       m_drive.kDriveKinematics,

//       frc2::PIDController(AutoConstants::kPXController, 0, 0),
//       frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController,

//       [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

//       {&m_drive});



//   // Reset odometry to the starting pose of the trajectory.
//   m_drive.ResetOdometry(m_SelectedTrajectory.InitialPose());

//   // no auto // I do not know why this says no auto although I have heard issues about this auto here
//   // I think that for actualy robot code that will include important things like shooting in 2023
//   // will have a "wrapper" command group to run multiple actions at once using parellel command group
//   return new frc2::ParallelCommandGroup(
//   frc2::SequentialCommandGroup(
//       std::move(swerveControllerCommand), frc2::InstantCommand([this] {m_drive.Drive(
//         0_mps,
//         0_mps,
//         units::radians_per_second_t(0),
//         true
//       );}, {&m_drive})));

//   //   return new frc2::ParallelCommandGroup(
//   // frc2::SequentialCommandGroup());

// }
