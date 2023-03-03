// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <numbers>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int turningEncoderPorts,
                           const double offset)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless, rev::CANSparkMax::IdleMode::kCoast),
      m_turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless, rev::CANSparkMax::IdleMode::kCoast),
      m_turningEncoder(turningEncoderPorts, offset),
      m_id(turningEncoderPorts), m_kOffset(m_turningEncoder.Get()) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveMotor.SetRPM2MPS(
      ModuleConstants::kDriveEncoderDistancePerPulse);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveMotor.GetRate()},
          frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    return {units::meter_t(m_driveMotor.GetDistance()), units::radian_t(m_turningMotor.GetDistance() + m_kOffset)};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {

  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_turningEncoder.Get()));

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      -m_driveMotor.GetRate(), state.speed.value());

  if(m_id == 1) {
    // printf("Rate: %5.2f, Asked:%5.2f, Drive: %5.2f\n", m_driveMotor.GetRate(), state.speed.value(), driveOutput);
  }

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(m_turningEncoder.Get()), state.angle.Radians());

//   if (m_id == 0) printf("Id: %d Get: %5.2f state: %5.2f | ", m_id, m_turningEncoder.Get(), state.angle.Radians().value());
//   if (m_id == 2) printf("Id: %d Get: %5.2f state: %5.2f\n", m_id, m_turningEncoder.Get(), state.angle.Radians().value());

  units::volt_t driveFF = m_driveFeedforward.Calculate(state.speed);

  // if (m_id == 1) {
  //   printf("TOut: %5.2f\n", turnOutput);
  // }

  // Set the motor outputs.
  // if(fabs(state.speed.value()) < 0.001) {
  //   m_driveMotor.Set(0);
  //   m_turningMotor.Set(0);
  // }
  // else {
  //   m_driveMotor.SetVoltage(units::volt_t(driveOutput) + driveFF);
  //   m_turningMotor.Set(turnOutput);
  // }
    // m_driveMotor.SetVoltage(units::volt_t(driveOutput) + driveFF);
    // m_driveMotor.Set(driveOutput);
    // m_turningMotor.Set(turnOutput);

    // if (m_id == 1) {
      // m_driveMotor.Set(driveOutput);
      // printf("T Out %5.2f\n", turnOutput);
    // }

  

      // if (state.speed.value() > 0.125)
        // m_driveMotor.Set((-driveOutput - driveFF.value()) / 12);
      if (state.speed.value() < 0.05)
      m_driveMotor.Set(0);
      else {
      if (m_id == 2)
      m_driveMotor.SetVoltage(units::volt_t(driveOutput) + driveFF);
      else 
      m_driveMotor.SetVoltage(units::volt_t(-driveOutput) - driveFF);
      }
      m_turningMotor.Set(turnOutput);


      // if (m_id == 2 || m_id == 3 || m_id == 1 || m_id == 4) {
      //   printf("ID: %d, Rate: %5.2f, dOut %5.2f\n", m_id, m_driveMotor.GetRate(), (double)driveFF + driveOutput);
      // }

    // if (m_id == 1 && i > 50) {
    //   // printf("Encoder: %5.2f - Drive Output %5.2f - turnOutput %5.2f\n", m_turningEncoder.Get(), driveOutput, turnOutput);
    //   i = 0;
    // }
    // i++;
}
