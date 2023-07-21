// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <math.h>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/DriverStation.h>
#include <frc/filter/MedianFilter.h>
#include "utils/cams/Limelight.h"

#include "Constants.h"

#define LAMBDA(x) [this] {return x;}

using namespace DriveConstants;
using namespace DriveConstants::CanIds;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  kFrontLeftTurningEncoderPorts,
                  kFrontLeftOffset},

      m_rearLeft{
          kRearLeftDriveMotorPort,       kRearLeftTurningMotorPort,
          kRearLeftTurningEncoderPorts,  kRearLeftOffset},

      m_frontRight{
          kFrontRightDriveMotorPort,       kFrontRightTurningMotorPort,
          kFrontRightTurningEncoderPorts,  kFrontRightOffset},

      m_rearRight{
          kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
          kRearRightTurningEncoderPorts,  kRearRightOffset},

      m_speed(DriveConstants::kMaxSpeed.value()), 

      m_odometry(kDriveKinematics, gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()}, frc::Pose2d()),
      m_poseEstimator(kDriveKinematics, gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()}, frc::Pose2d()),
      m_visionSystem(VisionSubsystem::GetInstance())
      // m_rightCam(m_visionSystem.GetRightCam())
                     {
                      m_field.SetRobotPose(frc::Pose2d(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad)));
                      frc::SmartDashboard::PutData(&m_field);
                      m_visionPoseRaw = frc::Translation2d();
                      m_calcVisionPose = frc::Pose2d();
                    }

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(-gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()});
  m_poseEstimator.Update(gyro.GetRot2d(),{m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()});

  // m_visionPoseRaw = hb::limeLight::GetBotPose2D();

  // m_calcVisionPose = frc::Pose2d(m_xFilter.Calculate(m_visionPoseRaw.X()), m_yFilter.Calculate(m_visionPoseRaw.Y()), gyro.GetRot2d());

  // m_field.SetRobotPose(m_odometry.GetPose());
  // m_field.SetRobotPose(m_calcVisionPose);

  if (m_visionSystem.GetPose().first.has_value()) {
    m_poseEstimator.AddVisionMeasurement(m_visionSystem.GetPose().second.value(), m_visionSystem.GetPose().first.value());
    m_calcVisionPose = m_visionSystem.GetPose().second.value();
    m_field.SetRobotPose(m_calcVisionPose);
  }

  // frc::SmartDashboard::PutNumber("LimeLight tx", hb::limeLight::GetX());
  // frc::SmartDashboard::PutNumber("LimeLight ta", hb::limeLight::GetA());

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, gyro.GetRot2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radian_t heading) {
    units::radians_per_second_t rot = units::radians_per_second_t(m_turnController.Calculate(gyro.GetRad(), heading));

    auto states = kDriveKinematics.ToSwerveModuleStates(
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      xSpeed, ySpeed, rot, gyro.GetRotation2d()
    ));

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  desiredStates[0].angle = ((desiredStates[0].angle * -1.0));
  desiredStates[1].angle = ((desiredStates[1].angle * -1.0));
  desiredStates[2].angle = ((desiredStates[2].angle * -1.0));
  desiredStates[3].angle = ((desiredStates[3].angle * -1.0));
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ZeroHeading() {
  gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return -gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(frc::Rotation2d(units::degree_t(gyro.GetAngle())), 
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                    m_frontRight.GetPosition(), m_rearRight.GetPosition()},
                    pose);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_rearRight.ResetEncoders();
}

void DriveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Swerve Drive");

  // builder.AddDoubleProperty("Heading", LAMBDA(gyro.GetRot2d().Degrees().value()), nullptr);
  
  // builder.AddDoubleProperty("FL V", LAMBDA(m_frontLeft.GetState().speed.value()), nullptr);
  // builder.AddDoubleProperty("FL A", LAMBDA(m_frontLeft.GetState().angle.Radians().value()), nullptr);

  // builder.AddDoubleProperty("FR V", LAMBDA(m_frontRight.GetState().speed.value()), nullptr);
  // builder.AddDoubleProperty("FR A", LAMBDA(m_frontRight.GetState().angle.Radians().value()), nullptr);  

  // builder.AddDoubleProperty("RL V", LAMBDA(m_rearLeft.GetState().speed.value()), nullptr);
  // builder.AddDoubleProperty("RL A", LAMBDA(m_rearLeft.GetState().angle.Radians().value()), nullptr);

  // builder.AddDoubleProperty("RR V", LAMBDA(m_rearRight.GetState().speed.value()), nullptr);
  // builder.AddDoubleProperty("RR A", LAMBDA(m_rearRight.GetState().angle.Radians().value()), nullptr);

  // builder.AddDoubleProperty("Battery", LAMBDA(frc::DriverStation::GetBatteryVoltage()), nullptr);

  // builder.AddDoubleProperty("FL DS", LAMBDA(m_frontLeft.GetDSetpoint()), nullptr);

  // builder.AddDoubleProperty("FL AO", LAMBDA(m_frontLeft.GetAppliedOut().first), nullptr);

  // builder.AddDoubleProperty("FL RV", LAMBDA(m_frontLeft.RequestedV()), nullptr);

  // builder.AddDoubleProperty("RCam X", LAMBDA(m_rightCam.GetLatestResult().GetBestTarget().GetYaw()), nullptr);

}