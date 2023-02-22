// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/SlewRateLimiter.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "commands/AutoRoutines.h"
#include "commands/DefaultDriveCMD.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  Auto m_auto;

  frc::SlewRateLimiter<units::scalar> m_speedLimitx{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_speedLimity{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_speedLimitz{3 / 1_s};

  double xSpeed = -m_speedLimitx.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY(), 0.05));
  double ySpeed = m_speedLimity.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX(), 0.05));
  double rot = m_speedLimitz.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), 0.05));

  bool m_FieldRelative = true;

  // The robot's subsystems
  DriveSubsystem m_drive;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
  frc::Trajectory m_SelectedTrajectory;
  int m_Routine;

  void ConfigureButtonBindings();
};
