// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/routines/MoveBackCMD.h"
#include "commands/routines/StopCMD.h"

class TestSeqCMD
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 TestSeqCMD> {
 public:
  TestSeqCMD(DriveSubsystem* drive);
};