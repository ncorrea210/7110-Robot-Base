#include "SwerveConfigs.h"
#include "Constants.h"

using namespace DriveConstants;
using namespace DriveConstants::CanIds;
using namespace ModuleConstants;

SwerveConfigs::SwerveConfigs() {
  m_FrontLeft.DriveMotorID = kFrontLeftDriveMotorPort;
  m_FrontLeft.TurnMotorID = kFrontLeftTurningMotorPort;
  m_FrontLeft.TurnEncoderID = kFrontLeftTurningEncoderPorts;
  m_FrontLeft.DriveVelocityRatio = kDriveEncoderVelocityRatio;
  m_FrontLeft.DrivePositionRatio = kDriveEncoderPositionRatio;
  m_FrontLeft.DriveIdle = rev::CANSparkMax::IdleMode::kCoast;
  m_FrontLeft.TurnIdle = rev::CANSparkMax::IdleMode::kBrake;
  m_FrontLeft.DriveCurrentLimit = 60;
  m_FrontLeft.TurnCurrentLimit = 20;
  m_FrontLeft.DriveP = kPDrive;
  m_FrontLeft.DriveI = kIDrive;
  m_FrontLeft.DriveD = kDDrive;
  m_FrontLeft.DriveFF = kFFDrive;
  m_FrontLeft.TurnP = kPTurn;
  m_FrontLeft.TurnI = kITurn;
  m_FrontLeft.TurnD = kDTurn;
  m_FrontLeft.TurnFF = kFFTurn;
  m_FrontLeft.TurnOffset = kFrontLeftOffset;

  m_FrontRight.DriveMotorID = kFrontRightDriveMotorPort;
  m_FrontRight.TurnMotorID = kFrontRightTurningMotorPort;
  m_FrontRight.TurnEncoderID = kFrontRightTurningEncoderPorts;
  m_FrontRight.DriveVelocityRatio = kDriveEncoderVelocityRatio;
  m_FrontRight.DrivePositionRatio = kDriveEncoderPositionRatio;
  m_FrontRight.DriveIdle = rev::CANSparkMax::IdleMode::kCoast;
  m_FrontRight.TurnIdle = rev::CANSparkMax::IdleMode::kBrake;
  m_FrontRight.DriveCurrentLimit = 60;
  m_FrontRight.TurnCurrentLimit = 20;
  m_FrontRight.DriveP = kPDrive;
  m_FrontRight.DriveI = kIDrive;
  m_FrontRight.DriveD = kDDrive;
  m_FrontRight.DriveFF = kFFDrive;
  m_FrontRight.TurnP = kPTurn;
  m_FrontRight.TurnI = kITurn;
  m_FrontRight.TurnD = kDTurn;
  m_FrontRight.TurnFF = kFFTurn;
  m_FrontRight.TurnOffset = kFrontRightOffset;

  m_RearLeft.DriveMotorID = kRearLeftDriveMotorPort;
  m_RearLeft.TurnMotorID = kRearLeftTurningMotorPort;
  m_RearLeft.TurnEncoderID = kRearLeftTurningEncoderPorts;
  m_RearLeft.DriveVelocityRatio = kDriveEncoderVelocityRatio;
  m_RearLeft.DrivePositionRatio = kDriveEncoderPositionRatio;
  m_RearLeft.DriveIdle = rev::CANSparkMax::IdleMode::kCoast;
  m_RearLeft.TurnIdle = rev::CANSparkMax::IdleMode::kBrake;
  m_RearLeft.DriveCurrentLimit = 60;
  m_RearLeft.TurnCurrentLimit = 20;
  m_RearLeft.DriveP = kPDrive;
  m_RearLeft.DriveI = kIDrive;
  m_RearLeft.DriveD = kDDrive;
  m_RearLeft.DriveFF = kFFDrive;
  m_RearLeft.TurnP = kPTurn;
  m_RearLeft.TurnI = kITurn;
  m_RearLeft.TurnD = kDTurn;
  m_RearLeft.TurnFF = kFFTurn;
  m_RearLeft.TurnOffset = kRearLeftOffset;

  m_RearRight.DriveMotorID = kRearRightDriveMotorPort;
  m_RearRight.TurnMotorID = kRearRightTurningMotorPort;
  m_RearRight.TurnEncoderID = kRearRightTurningEncoderPorts;
  m_RearRight.DriveVelocityRatio = kDriveEncoderVelocityRatio;
  m_RearRight.DrivePositionRatio = kDriveEncoderPositionRatio;
  m_RearRight.DriveIdle = rev::CANSparkMax::IdleMode::kCoast;
  m_RearRight.TurnIdle = rev::CANSparkMax::IdleMode::kBrake;
  m_RearRight.DriveCurrentLimit = 60;
  m_RearRight.TurnCurrentLimit = 20;
  m_RearRight.DriveP = kPDrive;
  m_RearRight.DriveI = kIDrive;
  m_RearRight.DriveD = kDDrive;
  m_RearRight.DriveFF = kFFDrive;
  m_RearRight.TurnP = kPTurn;
  m_RearRight.TurnI = kITurn;
  m_RearRight.TurnD = kDTurn;
  m_RearRight.TurnFF = kFFTurn;
  m_RearRight.TurnOffset = kRearRightOffset;

}

SwerveConfigs& SwerveConfigs::GetInstance() {
  static SwerveConfigs configs;
  return configs;
}

SwerveConfig SwerveConfigs::GetFrontLeft() {
  return m_FrontLeft;
}

SwerveConfig SwerveConfigs::GetFrontRight() {
  return m_FrontRight;
}

SwerveConfig SwerveConfigs::GetRearLeft() {
  return m_RearLeft;
}

SwerveConfig SwerveConfigs::GetRearRight() {
  return m_RearRight;
}