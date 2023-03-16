// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/routines/MoveBackCMD.h"


MoveBackCMD::MoveBackCMD(DriveSubsystem* drive) : m_drive(drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void MoveBackCMD::Initialize() {
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void MoveBackCMD::Execute() {
  m_drive->Drive(
    VecDrive(3, 0),
    0_rad
  );
}

// Called once the command ends or is interrupted.
void MoveBackCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool MoveBackCMD::IsFinished() {
  if (m_timer.Get().value() < 5) {
    return false;
  } else return true;
}
