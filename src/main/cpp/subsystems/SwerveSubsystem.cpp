#include "subsystems/SwerveSubsystem.h"

#include "SwerveConfigs.h"
#include "Constants.h"

SwerveSubsystem::SwerveSubsystem() :
  m_FrontLeft(SwerveConfigs::GetInstance().GetFrontLeft()),
  m_FrontRight(SwerveConfigs::GetInstance().GetFrontRight()), 
  m_RearLeft(SwerveConfigs::GetInstance().GetRearLeft()),
  m_RearRight(SwerveConfigs::GetInstance().GetRearRight()), 
  m_Gyro(0), 
  m_ModulePositions(
    m_FrontLeft.GetPosition(),
    m_FrontRight.GetPosition(),
    m_RearLeft.GetPosition(),
    m_RearRight.GetPosition()
  ), 
  m_Odometry(m_Kinematics, m_Gyro.GetRot2d(),
    m_ModulePositions, frc::Pose2d()) {}

void SwerveSubsystem::Periodic() {
  
  m_ModulePositions[0] = m_FrontLeft.GetPosition();
  m_ModulePositions[1] = m_FrontRight.GetPosition();
  m_ModulePositions[2] = m_RearLeft.GetPosition();
  m_ModulePositions[3] = m_RearRight.GetPosition();

  m_Odometry.Update(m_Gyro.GetRot2d(), m_ModulePositions);
}

void SwerveSubsystem::Drive(units::meters_per_second_t x, 
                            units::meters_per_second_t y,
                            units::radians_per_second_t rot, 
                            bool field) {
  
  auto states = m_Kinematics.ToSwerveModuleStates(
    field ? 
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      x, y, rot, m_Gyro.GetRot2d()) 
      : 
      frc::ChassisSpeeds(x, y, rot)
  );

  m_Kinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, rl, rr] = states;

  m_FrontLeft.SetDesiredState(fl);
  m_FrontRight.SetDesiredState(fr);
  m_RearLeft.SetDesiredState(rl);
  m_RearRight.SetDesiredState(rr);
}

void SwerveSubsystem::ZeroHeading(units::degree_t heading = 0_deg) {
  m_Gyro.SetPosition(heading);
} 

frc::Pose2d SwerveSubsystem::GetPose() {
  return m_Odometry.GetPose();
}

void SwerveSubsystem::ResetOdometry(const frc::Pose2d& pose) {
  m_Odometry.ResetPosition(m_Gyro.GetRot2d(), m_ModulePositions, pose);
}