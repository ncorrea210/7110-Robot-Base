// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/AnalogEncoder.h>
#include <frc/controller/PIDController.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>

#include <wpi/sendable/SendableBuilder.h>


struct ArmPosition{

  ArmPosition(uint16_t extension, uint16_t angle) {
    this->extension = extension;
    this->angle = angle;
  }

  uint16_t extension;
  uint16_t angle;

};

class ArmSubsystem : public frc2::SubsystemBase {
  public:
    ArmSubsystem();

    enum class State {
      kStow, 
      kMidCone, 
      kMidCubeConePickup, 
      kCubePickup, 
      kRunning 
    };

    /**
    * Will be called periodically whenever the CommandScheduler runs.
    */
    void Periodic() override;

    void CheckState();

    void StopMotors();

    void SetPosition(ArmPosition position);

    void Stow();

    void MidCone();

    void MidCubeConePickup();

    void CubePickup();

    State GetState() const;

    State GetTarget() const;

    int GetAngle() const;

    inline void RunExtension(double speed) {
      m_extension.Set(speed);
    }

    inline void RunActuator(double speed) {
      m_actuator.Set(speed);
    }

    void InitSendable(wpi::SendableBuilder& builder) override;

  private:

    State m_targetState;
    State m_actualState;

    rev::CANSparkMax m_extension;
    rev::SparkMaxRelativeEncoder m_extensionEncoder;
    rev::SparkMaxPIDController m_extensionController;

    frc::VictorSP m_actuator;
    frc::AnalogEncoder m_actuatorEncoder;
    frc::PIDController m_actuatorController;

    const ArmPosition m_stow;
    const ArmPosition m_coneMid;
    const ArmPosition m_cubeMidconePickup;
    // const ArmPosition m_conePickup;
    const ArmPosition m_cubePickup;

    ArmPosition m_target;
    
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};
