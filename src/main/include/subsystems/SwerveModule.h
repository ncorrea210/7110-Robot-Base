// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <numbers>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/length.h>
#include <units/voltage.h>
#include <units/velocity.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include "Constants.h"
#include "utils/NeoMotors.h"
#include "utils/CANCoder.h"

class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
                const int turningEncoderPorts, const double offset);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(std::numbers::pi * 10.0);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              std::numbers::pi * 100.0);  // radians per second squared

  hb::NeoMotor m_driveMotor;
  hb::NeoMotor m_turningMotor;
  hb::CANcode m_turningEncoder;
  
  int m_id;

  double i = 0;

  const double m_kOffset;

  frc2::PIDController m_drivePIDController{
      ModuleConstants::kPModuleDriveController, 0.01, 0.02};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPModuleTurningController,
      0.0,
      0.00,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{0.05_V, 2.67_V / 1_mps};
};
