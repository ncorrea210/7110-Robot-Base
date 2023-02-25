// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <functional>
#include <utility>
#include <numbers>
#include <cmath>

#include "subsystems/ExtensionSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DefaultExtendCMD
    : public frc2::CommandHelper<frc2::CommandBase, DefaultExtendCMD> {
 public:
  DefaultExtendCMD(ExtensionSubsystem* Extension, std::function<bool()> f, std::function<bool()> r);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 
 private:
  ExtensionSubsystem* m_Extension;
  std::function<bool()> m_f;
  std::function<bool()> m_r;
};
