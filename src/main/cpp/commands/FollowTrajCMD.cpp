// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowTrajCMD.h"

FollowTrajCMD::FollowTrajCMD(DriveSubsystem* drive, frc::Trajectory traj) : m_drive(drive), m_traj(traj) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void FollowTrajCMD::Initialize() {
  m_drive->m_gyro.SetPosition(m_traj.InitialPose().Rotation().Degrees());
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void FollowTrajCMD::Execute() {
  auto goal = m_traj.Sample(m_timer.Get());
  auto adjustedSpeeds = m_drive->GetController().Calculate(
    m_drive->GetPose(), goal, goal.pose.Rotation()
  );
  m_drive->SetModuleStates(m_drive->kDriveKinematics.ToSwerveModuleStates(adjustedSpeeds));
}

// Called once the command ends or is interrupted.
void FollowTrajCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool FollowTrajCMD::IsFinished() {
  if (m_traj.TotalTime() >= m_timer.Get() + 0.1_s)
  return true;
  else return false;
  
}
