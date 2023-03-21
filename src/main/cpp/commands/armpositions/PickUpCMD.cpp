// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/armpositions/PickUpCMD.h"

PickUpCMD::PickUpCMD(ExtensionSubsystem* extension, ActuatorSubsystem* actuator, bool cone) : m_extension(extension), m_actuator(actuator), m_cone(cone) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(extension);
  AddRequirements(actuator);
}

// Called when the command is initially scheduled.
void PickUpCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PickUpCMD::Execute() {
  m_actuator->SetPosition(2);
  if (m_cone) {
  m_extension->SetPos(100);
  } else if (!m_cone) {
  m_extension->SetPos(50);
  }
}

// Called once the command ends or is interrupted.
void PickUpCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PickUpCMD::IsFinished() {
  if (m_extension->GetPosition() == 100 && m_actuator->GetPosition() < 4) return true;
  else return false;
}
