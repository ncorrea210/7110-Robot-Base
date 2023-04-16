// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "commands/TelemetryCMD.h"

TelemetryCMD::TelemetryCMD(const std::initializer_list<hb::SubsystemData>& base) {
  // Use addRequirements() here to declare subsystem dependencies.
  for (hb::SubsystemData a : base) {
    m_data.insert(m_data.end(), a);
  }

}

// Called when the command is initially scheduled.
void TelemetryCMD::Initialize() {
    for (auto a : m_data) {
    frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab(a.name);
    for (auto b = a.telemetry.begin(); b != a.telemetry.end(); ++b) {
      tab.AddNumber(b->first, b->second);
    }
  }
}

// Called repeatedly when this Command is scheduled to run
void TelemetryCMD::Execute() {}

// Called once the command ends or is interrupted.
void TelemetryCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool TelemetryCMD::IsFinished() {
  return false;
}
