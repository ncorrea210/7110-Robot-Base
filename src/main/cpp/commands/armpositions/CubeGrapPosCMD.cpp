// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/armpositions/CubeGrapPosCMD.h"

CubeGrapPosCMD::CubeGrapPosCMD(ExtensionSubsystem* extension, ActuatorSubsystem* actuator) : m_extension(extension), m_actuator(actuator) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(extension);
  AddRequirements(actuator);
}

// Called when the command is initially scheduled.
void CubeGrapPosCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CubeGrapPosCMD::Execute() {
  m_extension->SetPos(50);
  m_actuator->SetPosition(2);
}

// Called once the command ends or is interrupted.
void CubeGrapPosCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool CubeGrapPosCMD::IsFinished() {
  if (m_extension->GetPosition() == 50 && m_actuator->GetPosition() < 4) return true;
  else return false;
}
