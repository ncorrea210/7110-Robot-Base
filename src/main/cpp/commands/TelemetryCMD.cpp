// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "commands/TelemetryCMD.h"

TelemetryCMD::TelemetryCMD(std::initializer_list<hb::Subsystem*> base) {
  // Use addRequirements() here to declare subsystem dependencies.
  for (hb::Subsystem* a : base) {
    m_subsys.insert(m_subsys.end(), a);
    AddRequirements(a); 
  }

}

// Called when the command is initially scheduled.
void TelemetryCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TelemetryCMD::Execute() {
  for (auto a : m_subsys) {
    frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab(a->GetName());
    for (auto b = a->GetTelemetry().begin(); b != a->GetTelemetry().end(); ++b) {
      tab.Add(b->first, b->second);
    }
  }
}

// Called once the command ends or is interrupted.
void TelemetryCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool TelemetryCMD::IsFinished() {
  return false;
}
