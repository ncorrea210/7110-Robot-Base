// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Rotation2d.h>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

#include "utils/swerve/SwerveModule.h"
#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int turningEncoderPorts,
                           const double offset)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningEncoder(turningEncoderPorts, offset),
      m_id(turningEncoderPorts),
      m_sparkDriveEncoder(m_driveMotor.GetEncoder()), 
      m_sparkTurnEncoder(m_turningMotor.GetEncoder()),
      m_tController(m_turningMotor.GetPIDController()), 
      m_dController(m_driveMotor.GetPIDController()) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  // m_driveMotor.SetRatio(
  //     ModuleConstants::kDriveEncoderDistancePerPulse);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  // m_turningPIDController.EnableContinuousInput(
  //     units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));

  m_driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_sparkTurnEncoder.SetPositionConversionFactor(ModuleConstants::kTurnEncoderRatio);
  m_sparkTurnEncoder.SetPosition(m_turningEncoder.Get());

  m_sparkDriveEncoder.SetVelocityConversionFactor(ModuleConstants::kDriveEncoderDistancePerPulse/60);
  m_sparkDriveEncoder.SetPositionConversionFactor(ModuleConstants::kDriveEncoderDistancePerPulse);

  m_driveMotor.SetClosedLoopRampRate(0);

  // m_turningController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  m_tController.SetPositionPIDWrappingEnabled(true);
  m_tController.SetPositionPIDWrappingMaxInput(std::numbers::pi);
  m_tController.SetPositionPIDWrappingMinInput(-std::numbers::pi);
  m_tController.SetP(m_turningController.GetP());
  m_tController.SetI(m_turningController.GetI());
  m_tController.SetD(m_turningController.GetD());
  m_tController.SetOutputRange(-1, 1);

  m_dController.SetP(kP);
  m_dController.SetI(kI);
  m_dController.SetD(kD);
  m_dController.SetFF(kV/12.0);
  m_dController.SetOutputRange(-1, 1);

  m_driveMotor.BurnFlash();
  m_turningMotor.BurnFlash();

}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_sparkDriveEncoder.GetVelocity()},
          frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    return {units::meter_t(-m_sparkDriveEncoder.GetPosition()), units::radian_t(-m_sparkTurnEncoder.GetPosition())};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {

  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_turningEncoder.Get()));


  if (fabs(state.speed.value()) < 0.01) {
    StopMotors();
  } else {
    // m_driveMotor.SetVoltage(units::volt_t(-driveOutput) - driveFF);
    // m_turningMotor.Set(turnOutput);
    m_dController.SetReference(state.speed.value(), rev::CANSparkMax::ControlType::kVelocity);
    m_tController.SetReference(state.angle.Radians().value(), rev::CANSparkMax::ControlType::kPosition);
  }
}

void SwerveModule::ResetEncoders() {
  // m_driveMotor.SetPosition(0);
}

void SwerveModule::StopMotors() {
  m_driveMotor.Set(0);
  m_turningMotor.Set(0);
}

units::celsius_t SwerveModule::GetDriveMotorTemp() {
  return units::celsius_t(m_driveMotor.GetMotorTemperature());
}

units::celsius_t SwerveModule::GetTurnMotorTemp() {
  return units::celsius_t(m_turningMotor.GetMotorTemperature());
}

std::pair<double, double> SwerveModule::GetAppliedOut() {
  return std::pair<double, double>{m_driveMotor.GetAppliedOutput(), m_turningMotor.GetAppliedOutput()};
}