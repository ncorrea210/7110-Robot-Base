// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <initializer_list>
#include <vector>
#include <string>
#include <unordered_map>

#include "utils/Subsystem.h"

/**
 * The telemetry command will use all subsystems and place the data for each subsystem onto its tab 
 */
class TelemetryCMD
    : public frc2::CommandHelper<frc2::CommandBase, TelemetryCMD> {
 public:

  TelemetryCMD(std::initializer_list<hb::Subsystem*> base);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  std::vector<hb::Subsystem*> m_subsys;

};
