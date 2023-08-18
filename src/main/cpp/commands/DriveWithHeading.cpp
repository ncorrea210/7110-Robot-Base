// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveWithHeading.h"

static bool InRange(double val, double target, double epsilon) {
    return (val > (target - epsilon) && val < (target + epsilon));
}

DriveWithHeading::DriveWithHeading(DriveSubsystem* drive, std::function<double()> x, std::function<double()> y, units::degree_t heading) : 
m_drive(drive), m_x(std::move(x)), m_y(std::move(y)), m_heading(heading) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
  m_controller.EnableContinuousInput(-180, 180);
}

// Called when the command is initially scheduled.
void DriveWithHeading::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveWithHeading::Execute() {
  auto rot = units::radians_per_second_t(m_controller.Calculate(m_drive->gyro.GetCompassHeading(), m_heading.value()));

  m_drive->Drive(
    units::meters_per_second_t(m_x() * DriveConstants::kMaxSpeed),
    units::meters_per_second_t(m_y() * DriveConstants::kMaxSpeed), 
    -rot, true
  );



}

// Called once the command ends or is interrupted.
void DriveWithHeading::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveWithHeading::IsFinished() {
  if (InRange(m_drive->gyro.GetCompassHeading(), m_heading.value(), 3)) return true;
  else return false;
}
