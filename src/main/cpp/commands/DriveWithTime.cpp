// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveWithTime.h"

DriveWithTime::DriveWithTime(DriveSubsystem* drive, units::meters_per_second_t x, units::meters_per_second_t y, 
                units::radians_per_second_t rot, units::second_t time, bool fieldRelative) : 
                m_drive(drive), m_x(x), m_y(y), m_rot(rot), m_time(time), m_fieldRelative(fieldRelative) {
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void DriveWithTime::Initialize() {
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void DriveWithTime::Execute() {
  m_drive->Drive(m_x, m_y, m_rot, m_fieldRelative);
}

// Called once the command ends or is interrupted.
void DriveWithTime::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveWithTime::IsFinished() {
  if (m_timer.Get() >= m_time) return true;
  else return false;
}
