// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autos/PlaceMidConeNLeaveSeqCMD.h"

#include "commands/FollowPPPathCMD.h"
#include "commands/armpositions/PlaceMidConeCMD.h"
#include "commands/ToLLTargetCMD.h"
#include "commands/OpenClampCMD.h"
#include "commands/armpositions/DrivePositionCMD.h"
#include "commands/routines/StopCMD.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceMidConeNLeaveSeqCMD::PlaceMidConeNLeaveSeqCMD(DriveSubsystem* drive, ExtensionSubsystem* extension, ActuatorSubsystem* actuator, ClampSubsystem* clamp) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
              PlaceMidConeCMD(extension, actuator), 
              frc2::WaitCommand(1_s), 
              OpenClampCMD(clamp), 
              DrivePositionCMD(extension, actuator), 
              FollowPPPathCMD(drive, "TPath"),
              StopCMD(drive)
              );
}