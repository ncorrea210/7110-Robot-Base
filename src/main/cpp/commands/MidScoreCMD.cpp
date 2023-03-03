// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MidScoreCMD.h"

MidScoreCMD::MidScoreCMD(ExtensionSubsystem* Extension, WinchSubsystem* Winch) : m_Extension(Extension), m_Winch(Winch) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(Extension);
  AddRequirements(Winch);
}

// Called when the command is initially scheduled.
void MidScoreCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MidScoreCMD::Execute() {
  m_Winch->SetPosition(0.1);
  m_Extension->SetPos(150);
}

// Called once the command ends or is interrupted.
void MidScoreCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool MidScoreCMD::IsFinished() {
  if (m_Winch->GetPosition() == 0.1 && m_Extension->GetPosition() == 150) {
  return true;
  } else return false;
}
