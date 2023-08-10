// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autos/TestAutoDWT.h"

#include <frc2/command/WaitCommand.h>

#include "commands/DriveWithTime.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TestAutoDWT::TestAutoDWT(DriveSubsystem* drive) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    DriveWithTime(drive, 1_mps, 0_mps, units::radians_per_second_t(0), 2_s, false)
  );
}
