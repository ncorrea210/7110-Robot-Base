// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/Trajectories.h"

using namespace hb;

frc::Trajectory Trajectories::MoveBack() {
    return frc::TrajectoryGenerator::GenerateTrajectory(
      {frc::Pose2d(0_m, 0_m, frc::Rotation2d(units::degree_t(180))),
      frc::Pose2d(-3_m, 0_m, frc::Rotation2d(units::degree_t(180)))},
      frc::TrajectoryConfig(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration)
    );
  }

  
