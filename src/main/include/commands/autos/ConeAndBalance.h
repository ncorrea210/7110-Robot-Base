// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file ConeAndBalance.h
 * @date 2023-08-19
 */

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClawSubsystem.h"

class ConeAndBalance
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ConeAndBalance> {
 public:
  ConeAndBalance(DriveSubsystem* drive, ArmSubsystem* arm, ClawSubsystem* claw);
};
