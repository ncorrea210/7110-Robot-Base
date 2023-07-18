// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angular_acceleration.h>
#include <numbers>


#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
    namespace CanIds{
    constexpr int kFrontLeftDriveMotorPort = 1;
    constexpr int kRearLeftDriveMotorPort = 3;
    constexpr int kFrontRightDriveMotorPort = 6;
    constexpr int kRearRightDriveMotorPort = 8;

    constexpr int kFrontLeftTurningMotorPort = 5;
    constexpr int kRearLeftTurningMotorPort = 7;
    constexpr int kFrontRightTurningMotorPort = 2;
    constexpr int kRearRightTurningMotorPort = 4;

    constexpr int kFrontLeftTurningEncoderPorts = 2;
    constexpr int kRearLeftTurningEncoderPorts = 1;
    constexpr int kFrontRightTurningEncoderPorts = 3;
    constexpr int kRearRightTurningEncoderPorts = 4;

    const int kPidgeonID = 0;
    } // namespace CanIds

constexpr double kFrontLeftOffset = 41.31; //encoder 2
constexpr double kRearLeftOffset =  -42.71; //encoder 1
constexpr double kFrontRightOffset = 107.48; //encoder 3
constexpr double kRearRightOffset = -168.75; //encoder 4

constexpr auto kMaxSpeed = 1.5_mps;
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(0.5 * std::numbers::pi);
constexpr auto kMaxAngularAcceleration = units::radians_per_second_squared_t(2 * std::numbers::pi);

constexpr auto kTrackWidth = 0.31369_m;
constexpr auto kTrackLength = 0.31369_m;


}  // namespace DriveConstants

namespace ModuleConstants {
constexpr double kGearRatio = 1/6.75;
constexpr double kWheelDiameterMeters = 0.05092958;
constexpr double kDriveEncoderDistancePerPulse =
kGearRatio * 2 * std::numbers::pi * kWheelDiameterMeters;
constexpr double kDriveEncoderVelocityRatio = kDriveEncoderDistancePerPulse;
constexpr double kDriveEncoderPositionRatio = kDriveEncoderDistancePerPulse;

constexpr double kTurnRatio = 7.0/150.0;
constexpr double kTurnEncoderRatio = kTurnRatio * 2.0 * std::numbers::pi;

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 0.75;

constexpr double kPDrive = 1.0;
constexpr double kIDrive = 0;
constexpr double kDDrive = 0;
constexpr double kFFDrive = 2.67;

constexpr double kPTurn = 0.75;
constexpr double kITurn = 0;
constexpr double kDTurn = 0.004;
constexpr double kFFTurn = 0;
}  // namespace ModuleConstants

namespace AutoConstants {
using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

constexpr auto kMaxSpeed = units::meters_per_second_t(1.5);
constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(1.5);
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
constexpr auto kMaxAngularAcceleration = units::unit_t<radians_per_second_squared_t>(3.142);

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;


extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
