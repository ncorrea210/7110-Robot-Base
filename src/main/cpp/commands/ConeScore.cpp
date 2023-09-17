// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ConeScore.h"

ConeScore::ConeScore(DriveSubsystem* drive, std::function<double()> forwards) : 
  m_drive(drive), m_forwards(std::move(forwards)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void ConeScore::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ConeScore::Execute() {
  units::meters_per_second_t forwards = units::meters_per_second_t(m_forwards() * AutoConstants::kMaxSpeed);
  units::meters_per_second_t horizontal = units::meters_per_second_t(m_horizontalController.Calculate(hb::LimeLight::GetX(), 0));
  
  
  m_drive->Drive(
    forwards,
    -horizontal,
    0_rad_per_s,
    true
  );
}

// Called once the command ends or is interrupted.
void ConeScore::End(bool interrupted) {}

// Returns true when the command should end.
bool ConeScore::IsFinished() {
  if (m_drive->GetTarget() != DriveSubsystem::Target::kCone) {
    return true;
  }
  if (!hb::LimeLight::HasTarget()) {
    return true;
  }

  return false;
  
}
