// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PickUpAngleCMD.h"

PickUpAngleCMD::PickUpAngleCMD(WinchSubsystem* Winch) : m_Winch(Winch) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(Winch);
}

// Called when the command is initially scheduled.
void PickUpAngleCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PickUpAngleCMD::Execute() {
  m_Winch->SetPosition(0.11);
}

// Called once the command ends or is interrupted.
void PickUpAngleCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PickUpAngleCMD::IsFinished() {
  if (m_Winch->GetPosition() == 0.11) 
  return true;
  else 
  return false;
}
