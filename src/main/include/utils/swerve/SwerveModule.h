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
#include "utils/swerve/CANCoder.h"

class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
                const int turningEncoderPorts, const double offset);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  inline double GetDSetpoint() const {
    return m_driveMotor.Get();
  }

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

  void StopMotors();

  units::celsius_t GetDriveMotorTemp();

  units::celsius_t GetTurnMotorTemp(); 

  /**
   * @returns the applied output of the drive motor and the turn motor in order first to second respectively
  */
  std::pair<double, double> GetAppliedOut();


 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(std::numbers::pi * 4);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              std::numbers::pi * 100.0);  // radians per second squared

  // hb::NeoMotor m_driveMotor;
  // hb::NeoMotor m_turningMotor;

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;
  rev::SparkMaxRelativeEncoder m_sparkDriveEncoder;
  rev::SparkMaxRelativeEncoder m_sparkTurnEncoder;
  rev::SparkMaxPIDController m_tController;
  rev::SparkMaxPIDController m_dController;

  hb::CANcode m_turningEncoder;
  
  int m_id;


  double kP = 0.175;
  double kI = 0;
  double kD = 0.02;

  frc2::PIDController m_drivePIDController{
      0.825, 0.0, 0.03};
//   frc::ProfiledPIDController<units::radians> m_turningPIDController{
//       ModuleConstants::kPModuleTurningController,
//       0.0,
//       0.00,
//       {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

    // units::compound_unit<units::volt_t, units::inverse<units::meters_per_second_t>> kV = 2.67_V / 1_mps;

    double kV = 2.67;
    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{0_V, units::volt_t(2.67) / 1_mps};

      frc::ProfiledPIDController<units::radians> m_turningPIDController{
      0.75,
      0.0,
      /*0.011*/0.004,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

    frc2::PIDController m_turningController {1.0, 0.0, 0};

    
};
