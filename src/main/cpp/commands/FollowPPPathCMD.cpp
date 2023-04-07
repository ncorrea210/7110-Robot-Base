// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowPPPathCMD.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;

FollowPPPathCMD::FollowPPPathCMD(DriveSubsystem* drive, std::string path) : m_drive(drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
  m_traj = PathPlanner::loadPath(path, PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration), false);
  frc::SmartDashboard::PutNumber("Total Time", m_traj.getTotalTime().value());
}

// Called when the command is initially scheduled.
void FollowPPPathCMD::Initialize() {
  auto initState = m_traj.getInitialState();
  auto initPose = initState.pose;
  m_drive->ResetOdometry({initPose.Translation(), initState.holonomicRotation});
  m_drive->m_gyro.SetPosition(initState.holonomicRotation.Degrees());
  m_drive->ResetEncoders();
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void FollowPPPathCMD::Execute() {
  auto state = m_traj.sample(m_timer.Get());
  frc::Trajectory::State stateW = state.asWPILibState();
  m_drive->SetModuleStates(m_drive->kDriveKinematics.ToSwerveModuleStates(
    m_drive->GetController().Calculate(m_drive->GetPose(), stateW, state.holonomicRotation)));
  frc::SmartDashboard::PutNumber("Cur Time", m_timer.Get().value());
}

// Called once the command ends or is interrupted.
void FollowPPPathCMD::End(bool interrupted) {
  m_timer.Stop();
  m_timer.Reset();
}

// Returns true when the command should end.
bool FollowPPPathCMD::IsFinished() {
  if (m_traj.getTotalTime().value() < (m_timer.Get().value() + 0.1)) {
    m_timer.Stop();
    m_timer.Reset();
    return true;
  } 
  else return false;
}
