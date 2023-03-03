// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DefaultPositionCMD.h"

DefaultPositionCMD::DefaultPositionCMD(ExtensionSubsystem* Extension, WinchSubsystem* Winch) : m_Extension(Extension), m_Winch(Winch){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(Extension);
  AddRequirements(Winch);
}

// Called when the command is initially scheduled.
void DefaultPositionCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultPositionCMD::Execute() {
  m_Winch->SetPosition(0.7);
  m_Extension->ZeroExtension();
}

// Called once the command ends or is interrupted.
void DefaultPositionCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool DefaultPositionCMD::IsFinished() {
  if (m_Winch->GetPosition() < 0.7 && m_Extension->GetPosition() == 0) {
    return true;
  } else return false;
}
