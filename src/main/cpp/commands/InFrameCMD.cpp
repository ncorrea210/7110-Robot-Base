// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/InFrameCMD.h"

InFrameCMD::InFrameCMD(WinchSubsystem* Winch, ExtensionSubsystem* Extension) : m_Winch(Winch), m_Extension(Extension) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(Winch);
  AddRequirements(Extension);
}

// Called when the command is initially scheduled.
void InFrameCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void InFrameCMD::Execute() {
  m_Winch->SetPosition(0.03);
  m_Extension->ZeroExtension();
}

// Called once the command ends or is interrupted.
void InFrameCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool InFrameCMD::IsFinished() {
  if (m_Winch->GetPosition() == 0.04 && m_Extension->GetPosition() == 0) {
    return true;
  } else return false;
}
