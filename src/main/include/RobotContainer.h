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
#include <frc/DutyCycleEncoder.h>
#include <frc/PowerDistribution.h>


#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ExtensionSubsystem.h"
#include "subsystems/WinchSubsystem.h"
#include "commands/AutoRoutines.h"
#include "commands/DefaultDriveCMD.h"
#include "subsystems/ClampSubsystem.h"
#include "commands/SetFarPositionCMD.h"
#include "commands/CloseClawCMD.h"
#include "commands/OpenClawCMD.h"
#include "commands/InFrameCMD.h"
#include "commands/CloseCubeCMD.h"
#include "commands/DefaultPositionCMD.h"
#include "commands/MidScoreCMD.h"

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
  frc::XboxController m_operatorController{1};

  // The robot's subsystems and commands are defined here...

  Auto m_auto;

  frc::SlewRateLimiter<units::scalar> m_speedLimitx{1.5 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_speedLimity{1.5 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_speedLimitz{1.5 / 1_s};

  // The robot's subsystems
  DriveSubsystem m_drive;
  WinchSubsystem m_Winch{&m_MainEncoder, &m_PDP};
  ExtensionSubsystem m_Extension{&m_MainEncoder, &m_PDP};
  ClampSubsystem m_clamp{&m_PDP};

  SetFarPositionCMD SetFar{&m_Extension, &m_Winch};
  CloseClawCMD CloseClaw{&m_clamp};
  OpenClawCMD OpenClaw{&m_clamp};
  InFrameCMD InFrame{&m_Winch, &m_Extension};
  CloseCubeCMD CloseCube{&m_clamp};
  DefaultPositionCMD DefaultPosition{&m_Extension, &m_Winch};
  MidScoreCMD MidScore{&m_Extension, &m_Winch};

  frc::DutyCycleEncoder m_MainEncoder{0};
  frc::PowerDistribution m_PDP{0, frc::PowerDistribution::ModuleType::kCTRE};

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
  frc::Trajectory m_SelectedTrajectory;
  int m_Routine;

  void ConfigureButtonBindings();
};
