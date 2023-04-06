// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autos/BalanceSeqCMD.h"
#include "commands/FollowPPPathCMD.h"
#include "commands/routines/StopCMD.h"
#include "commands/BalanceCMD.h"
#include "commands/armpositions/ScoreCMD.h"
#include "commands/OpenClampCMD.h"
#include "commands/armpositions/InFrameCMD.h"
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
BalanceSeqCMD::BalanceSeqCMD(DriveSubsystem* drive, ExtensionSubsystem* extension, ActuatorSubsystem* actuator, ClampSubsystem* clamp) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    ScoreCMD(extension, actuator, [] {return false;}),
    frc2::WaitCommand(0.25_s),
    OpenClampCMD(clamp),
    InFrameCMD(extension, actuator), 
    frc2::WaitCommand(0.25_s), 
    FollowPPPathCMD(drive, "BPath"), 
    BalanceCMD(drive), 
    StopCMD(drive));
}
