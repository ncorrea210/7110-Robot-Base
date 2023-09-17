// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autos/ConeAndBalance.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>

#include "commands/DriveWithTime.h"
#include "commands/Balance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ConeAndBalance::ConeAndBalance(DriveSubsystem* drive, ArmSubsystem* arm, ClawSubsystem* claw) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    frc2::InstantCommand([drive] {drive->gyro.SetPosition(180_deg);}),
    frc2::InstantCommand([arm] {arm->MidCone();}),
    frc2::WaitCommand(2_s), 
    frc2::InstantCommand([claw] {claw->Run(0.5);}),
    frc2::WaitCommand(0.7_s),
    frc2::InstantCommand([claw] {claw->Run(0);}),
    frc2::InstantCommand([arm] {arm->Stow();}),
    DriveWithTime(drive, -1_mps, 0_mps, units::radians_per_second_t(0), 2.75_s, false),
    frc2::WaitCommand(1_s),
    DriveWithTime(drive, 1_mps, 0_mps, units::radians_per_second_t(0), 2_s, false),
    Balance(drive)
  );
}
