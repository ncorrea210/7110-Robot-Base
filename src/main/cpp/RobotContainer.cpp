// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "RobotContainer.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/ConditionalCommand.h>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <utility>
#include <cmath>
#include <numbers>
#include <memory>
#include <unordered_map>

#include <units/angle.h>
#include <units/velocity.h>

#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

#include "commands/FollowPPPathCMD.h"
#include "commands/autos/CubeAndBalance.h"
#include "commands/autos/ConeAndBalance.h"
#include "commands/DriveWithTime.h"
#include "commands/Balance.h"
#include "commands/DriveWithHeading.h"
#include "commands/autos/TestCMD.h"
#include "utils/cams/Limelight.h"
#include "subsystems/DriveSubsystem.h"
#include "Constants.h"
#include "utils/Util.h"

using namespace DriveConstants;

// Drive macros ensure that all outputs stay the same
#define X_OUT [this] {return -m_speedLimitx.Calculate(frc::ApplyDeadband(hb::sgn(m_driverController.GetLeftY()) * pow(m_driverController.GetLeftY(), 2), 0.01));}
#define Y_OUT [this] {return -m_speedLimity.Calculate(frc::ApplyDeadband(hb::sgn(m_driverController.GetLeftX()) * pow(m_driverController.GetLeftX(), 2), 0.01));}
#define ROT_OUT [this] {return -frc::ApplyDeadband(hb::sgn(m_driverController.GetRightX()) * pow(m_driverController.GetRightX(), 2), 0.025) * DriveConstants::kMaxAngularSpeed.value();}

RobotContainer::RobotContainer() {

  /**
   * Adding options to chooser should be done as such
   * m_chooser.AddOption("Auto Name", &Auto);
   * Where auto has already been declared in RobotContainer.h
  */

  // m_chooser.AddOption("Test1", new FollowPPPathCMD(&m_drive, "Straight Line"));
  // m_chooser.AddOption("L Path", new FollowPPPathCMD(&m_drive, "BackForth"));
  // m_chooser.AddOption("T Drive With Time", new TestAutoDWT(&m_drive));
  m_chooser.AddOption("CommOut", new DriveWithTime(&m_drive, -2_mps, 0_mps, 0_rad_per_s, 1.25_s, false));
  m_chooser.AddOption("CubeNBalance", new CubeAndBalance(&m_drive, &m_arm, &m_claw));
  m_chooser.AddOption("ConeNBalance", new ConeAndBalance(&m_drive, &m_arm, &m_claw));
  m_chooser.AddOption("PathPlanner Test", new FollowPPPathCMD(&m_drive, "BackForth", false));
  m_chooser.AddOption("RedPathTest", new FollowPPPathCMD(&m_drive, "Red", true));
  m_chooser.AddOption("RedRotateBalance", new FollowPPPathCMD(&m_drive, "RotateBalance", true));
  // m_chooser.AddOption("Test", new TestCMD(&m_drive));

  m_autoChooser.AddOption("Pick and Place", "BackForth");

  frc::SmartDashboard::PutData("Auto Options", &m_autoChooser);
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);

  frc::SmartDashboard::PutData("Arm", &m_arm);
  frc::SmartDashboard::PutData("Claw", &m_claw);
  frc::SmartDashboard::PutData("Swerve", &m_drive);
  frc::SmartDashboard::PutData("Vision", &m_vision);
  frc::SmartDashboard::PutData("PDP", &m_pdp);

  // Configure the button bindings
  ConfigureButtonBindings();

  //Default subsystem commands are defined in this section
  // m_drive.SetDefaultCommand(DefaultDriveCMD(&m_drive, 
  //       [this] {return (m_speedLimitx.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY() < 0 ? -(m_driverController.GetLeftY() * m_driverController.GetLeftY()) : (m_driverController.GetLeftY() * m_driverController.GetLeftY()), 0.01)));}, 
  //       [this] {return -m_speedLimity.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX() < 0 ? -(m_driverController.GetLeftX() * m_driverController.GetLeftX()) : (m_driverController.GetLeftX() * m_driverController.GetLeftX()), 0.01));},
  //       [this] {return -(frc::ApplyDeadband(m_driverController.GetRightX() < 0 ? -(m_driverController.GetRightX() * m_driverController.GetRightX()) : (m_driverController.GetRightX() * m_driverController.GetRightX()), 0.025) * (double)DriveConstants::kMaxAngularSpeed);}, 
  //       [this] {return true;},
  //       [this] {return m_drive.GetSpeed().value();}));

  // New version with macros to test
  m_drive.SetDefaultCommand(DefaultDriveCMD(
    &m_drive, 
    X_OUT, 
    Y_OUT, 
    ROT_OUT,
    LAMBDA(true),
    LAMBDA(DriveConstants::kMaxSpeed.value() * (m_triggerLimit.Calculate(m_driverController.GetRightTriggerAxis()) * 0.625 + 0.375))
  )); 

}

void RobotContainer::ConfigureButtonBindings() {
  /**
   *  When binding buttons use this as the template
   *  frc::JoystickButton(&m_controller, frc::XboxController::Button::kButton).WhenPressed(Command);
   *  TODO: test command xbox controller implementation(found below) and see if it works
  */

  ConfigureDriverButtons();
  ConfigureOperatorButtons();

}

void RobotContainer::ConfigureDriverButtons() {
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY).WhenPressed([this] {m_arm.MidCone();});
  
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA).WhenPressed([this] {m_arm.Stow();});

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB).WhenPressed([this] {m_arm.MidCubeConePickup();});

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).WhenPressed([this] {m_arm.CubePickup();});

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper).WhenPressed([this] {m_claw.Run(0.5);})
    .WhenReleased([this] {m_claw.Run(0);});

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhenPressed([this] {m_claw.Run(-0.5);})
    .WhenReleased([this] {m_claw.Run(0);});

  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftStick).WhenPressed([this] {m_arm.MsMaiCar();});

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftStick).WhenPressed(frc2::InstantCommand([this] {m_drive.ToggleVision();}));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightStick).WhenPressed(frc2::InstantCommand
      ([] {hb::LimeLight::SetPipeline(hb::LimeLight::GetPipeline() == hb::LimeLight::Pipeline::kAprilTag ? 
      hb::LimeLight::Pipeline::kRetroReflective : hb::LimeLight::Pipeline::kAprilTag);}));
  
  /** @section POV Headings **/

  frc2::POVButton(&m_driverController, 0).WhenPressed(DriveWithHeading(
    &m_drive,
    X_OUT, 
    Y_OUT,
    0_deg
  ));

  frc2::POVButton(&m_driverController, 45).WhenPressed(DriveWithHeading(
    &m_drive, 
    X_OUT,
    Y_OUT, 
    45_deg
  ));  

  frc2::POVButton(&m_driverController, 90).WhenPressed(DriveWithHeading(
    &m_drive, 
    X_OUT,
    Y_OUT, 
    90_deg
  ));

  frc2::POVButton(&m_driverController, 135).WhenPressed(DriveWithHeading(
    &m_drive, 
    X_OUT,
    Y_OUT, 
    135_deg
  ));

  frc2::POVButton(&m_driverController, 180).WhenPressed(DriveWithHeading(
    &m_drive, 
    X_OUT,
    Y_OUT, 
    180_deg
  ));

  frc2::POVButton(&m_driverController, 225).WhenPressed(DriveWithHeading(
    &m_drive, 
    X_OUT,
    Y_OUT, 
    225_deg
  ));

  frc2::POVButton(&m_driverController, 270).WhenPressed(DriveWithHeading(
    &m_drive, 
    X_OUT,
    Y_OUT, 
    270_deg
  ));

  frc2::POVButton(&m_driverController, 315).WhenPressed(DriveWithHeading(
    &m_drive, 
    X_OUT,
    Y_OUT, 
    315_deg
  ));
  
  /** @section endsection **/

}

void RobotContainer::ConfigureOperatorButtons() {
  #ifdef OPERATORCONTROLLER

    // Emergency Shutoff
    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kB).WhenPressed(frc2::InstantCommand([this] {m_arm.Homing(false);}));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kA).WhenPressed(frc2::InstantCommand([this] {m_arm.Homing(true);}));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kX).WhenPressed(frc2::InstantCommand([this] {m_claw.Enable(false);}));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kY).WhenPressed(frc2::InstantCommand([this] {m_claw.Enable(true);}));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kRightBumper).WhenPressed(frc2::InstantCommand([this] {m_claw.Run(-0.5);})).WhenReleased(
      frc2::InstantCommand([this] {m_claw.Run(0.5);}));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kLeftBumper).WhenPressed(frc2::InstantCommand([this] {m_claw.Run(0.5);})).WhenReleased(
      frc2::InstantCommand([this] {m_claw.Run(0.0);}));

  #endif
}


frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}

frc2::CommandPtr RobotContainer::GetAuto() {

  auto traj = pathplanner::PathPlanner::loadPath(m_autoChooser.GetSelected(), AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  pathplanner::SwerveAutoBuilder builder{
    [this] {return m_drive.GetPose();}, 
    [this] (auto pose) {m_drive.SetPose(pose);},
    pathplanner::PIDConstants(AutoConstants::kPXController, 0, 0), 
    pathplanner::PIDConstants(AutoConstants::kPThetaController, 0, 0),
    [this] (auto speeds) {m_drive.DriveFieldRelative(speeds);}, 
    eventMap, 
    {&m_drive},
    true
  };

  return builder.fullAuto(traj);

}