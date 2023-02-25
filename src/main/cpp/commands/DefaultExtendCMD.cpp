// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DefaultExtendCMD.h"

DefaultExtendCMD::DefaultExtendCMD(ExtensionSubsystem* Extension, std::function<bool()> f, std::function<bool()> r)
                                    : m_Extension(Extension), m_f(std::move(f)), m_r(std::move(r)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(Extension);
}

// Called when the command is initially scheduled.
void DefaultExtendCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultExtendCMD::Execute() {
  if(m_Extension->GetPosition() * std::cos((m_Extension->GetAngle() / 180) * std::numbers::pi) > (28.0 + 48.0) /*Max extension out of frame from base of arm*/) {
    m_Extension->RunExtension(-1.0);
  } else if (m_f() == true) {
    m_Extension->RunExtension(1.0);
  } else if (m_r() == true) {
    m_Extension->RunExtension(-1.0);
  } else return;
}

// Called once the command ends or is interrupted.
void DefaultExtendCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool DefaultExtendCMD::IsFinished() {
  return false;
}
