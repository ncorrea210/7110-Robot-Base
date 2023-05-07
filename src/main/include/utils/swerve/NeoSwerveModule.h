#pragma once

#include <units/length.h>
#include <units/voltage.h>
#include <units/temperature.h>
#include <rev/CANSparkMax.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/PIDController.h>

#include <functional>
#include <utility>

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

    units::celsius_t GetDriveTemp();

    units::celsius_t GetTurnTemp();

  private:
    rev::CANSparkMax m_driveMotor;
    rev::SparkMaxRelativeEncoder m_driveEncoder;
    rev::SparkMaxPIDController m_drivePID;

    rev::CANSparkMax m_turnMotor;
    frc2::PIDController m_turnPID;
    hb::CANcode m_turnEncoder;
};