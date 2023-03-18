// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autos/BalanceSeqCMD.h"
#include "commands/FollowPPPathCMD.h"
#include "commands/routines/StopCMD.h"
#include <frc2/command/SwerveControllerCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
BalanceSeqCMD::BalanceSeqCMD(DriveSubsystem* drive) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(FollowPPPathCMD(drive, "BPath"), StopCMD(drive));
}
