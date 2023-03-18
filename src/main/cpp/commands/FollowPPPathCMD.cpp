// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowPPPathCMD.h"
#include "Constants.h"

#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;

FollowPPPathCMD::FollowPPPathCMD(DriveSubsystem* drive, std::string path) : m_drive(drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
  m_traj = PathPlanner::loadPath(path, PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration), false);
}

// Called when the command is initially scheduled.
void FollowPPPathCMD::Initialize() {
  auto initState = m_traj.getInitialState();
  auto initPose = initState.pose;
  m_drive->m_gyro.SetPosition(initPose.Rotation().Degrees());
  m_drive->ResetOdometry({initPose.Translation(), initState.holonomicRotation});
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void FollowPPPathCMD::Execute() {
  auto state = m_traj.sample(m_timer.Get());
  m_drive->SetModuleStates(m_drive->kDriveKinematics.ToSwerveModuleStates(
    m_drive->GetController().Calculate(m_drive->GetPose(), state.asWPILibState(), state.holonomicRotation)));
}

// Called once the command ends or is interrupted.
void FollowPPPathCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool FollowPPPathCMD::IsFinished() {
  if (m_traj.getTotalTime() >= m_timer.Get() + 0.1_s)
  return true;
  else return false;
}
