#pragma once

#include <units/length.h>
#include <units/voltage.h>
#include <rev/CANSparkMax.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/PIDController.h>

#include "Config.h"
#include "CANCoder.h"

class NeoSwerveModule {
  public:
    NeoSwerveModule(SwerveConfig config);

    frc::SwerveModuleState GetState();

    frc::SwerveModulePosition GetPosition();

    void StopMotors();

    void SetDesiredState(const frc::SwerveModuleState& state);

    void ResetEncoders();

  private:
    rev::CANSparkMax m_driveMotor;
    rev::SparkMaxRelativeEncoder m_driveEncoder;
    rev::SparkMaxPIDController m_drivePID;

    rev::CANSparkMax m_turnMotor;
    frc2::PIDController m_turnPID;
    hb::CANcode m_turnEncoder;
};