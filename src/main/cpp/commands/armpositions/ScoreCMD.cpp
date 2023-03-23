// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/armpositions/ScoreCMD.h"
#include <utility>

ScoreCMD::ScoreCMD(ExtensionSubsystem* extension, ActuatorSubsystem* actuator, std::function<bool()> cone) : 
                  m_extension(extension), m_actuator(actuator), m_cone(std::move(cone)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(extension);
  AddRequirements(actuator);
}

// Called when the command is initially scheduled.
void ScoreCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ScoreCMD::Execute() {
  m_actuator->SetPosition(2);
  if (m_cone()) {
    m_extension->SetMax();
  } else if (!m_cone()) {
    m_extension->SetPos(100);
  } 
}

// Called once the command ends or is interrupted.
void ScoreCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool ScoreCMD::IsFinished() {
  if (m_cone() && m_actuator->GetPosition() < 4 && m_extension->GetPosition() == 158) return true;
  else if (!m_cone() && m_actuator->GetPosition() < 4 && m_extension->GetPosition() == 100) return true;
  else return false;
}
