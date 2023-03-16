// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/routines/StopCMD.h"
#include <units/angular_velocity.h>
#include <units/velocity.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
StopCMD::StopCMD(DriveSubsystem* drive) : m_drive(drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void StopCMD::Initialize() {
  m_drive->Drive(
    0_mps,
    0_mps,
    0_rad_per_s,
    true
  );
}
