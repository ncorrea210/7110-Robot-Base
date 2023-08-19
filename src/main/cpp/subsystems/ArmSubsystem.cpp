// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>

#include "utils/Util.h"

#define EPSILON_EXTENSION 1.5
#define EPSILON_ANGLE 2
#define SWITCH_CHECK 75

ArmSubsystem::ArmSubsystem() : 
m_targetState(State::kStow),
m_actualState(State::kRunning), 
m_extension(9, rev::CANSparkMax::MotorType::kBrushless),
m_extensionEncoder(m_extension.GetEncoder()),
m_extensionController(m_extension.GetPIDController()),
m_actuator(1),
m_actuatorEncoder(0),
m_actuatorController(1, 0, 0),
m_limitSwitch(0),
m_stow(0, 98), 
m_coneMid(160, 2),
m_cubeMidconePickup(100, 2),
m_cubePickup(50, 2),
m_MsMaiCar(0, 2),
m_target(m_stow)
{

    m_extensionController.SetP(0.03);
    m_extensionController.SetOutputRange(-0.5, 0.6);
    
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {

    if (SwitchHigh()) {
        m_extensionEncoder.SetPosition(160);
    }

    if (SwitchLow()) {
        m_extensionEncoder.SetPosition(0);
    }

    CheckState();

    switch (m_targetState) {
        case State::kStow:
        m_target = m_stow;
        break;
        
        case State::kMidCone:
        m_target = m_coneMid;
        break;

        case State::kMidCubeConePickup:
        m_target = m_cubeMidconePickup;
        break;

        case State::kCubePickup:
        m_target = m_cubePickup;
        break;

        case State::kMsMaiCar:
        m_target = m_MsMaiCar;
        break;

        case State::kRunning:
        break;
    }
    
    if (m_actualState == m_targetState) {
        StopMotors();
        return;
    }

    if (!hb::InRange(m_extensionEncoder.GetPosition(), m_target.extension, EPSILON_EXTENSION)) {
    m_extensionController.SetReference(m_target.extension, rev::CANSparkMax::ControlType::kPosition);
    } else m_extension.Set(0);

    if (!hb::InRange(GetAngle(), m_target.angle, EPSILON_ANGLE)) {
        m_actuator.Set(m_actuatorController.Calculate(GetAngle(), m_target.angle));
    } else m_actuator.Set(0);
    
}

void ArmSubsystem::StopMotors() {
    m_extension.Set(0);
    m_actuator.Set(0);
}

void ArmSubsystem::SetPosition(ArmPosition position) {
    m_target = position;
}

void ArmSubsystem::Stow() {
    m_targetState = State::kStow;
}

void ArmSubsystem::MidCone() {
    m_targetState = State::kMidCone;
}

void ArmSubsystem::MidCubeConePickup() {
    m_targetState = State::kMidCubeConePickup;
}

void ArmSubsystem::CubePickup() {
    m_targetState = State::kCubePickup;
}

void ArmSubsystem::MsMaiCar() {
    m_targetState = State::kMsMaiCar;
}

ArmSubsystem::State ArmSubsystem::GetState() const {
    return m_actualState;
}

ArmSubsystem::State ArmSubsystem::GetTarget() const {
    return m_targetState;
}

int ArmSubsystem::GetAngle() const {
    return std::lround(100 * (((100 * m_actuatorEncoder.Get().value()) - 3) / 45));
}

bool ArmSubsystem::SwitchLow() const {
    if (m_extensionEncoder.GetPosition() < 75 && !m_limitSwitch.Get()) 
        return true;
    else return false;
}

bool ArmSubsystem::SwitchHigh() const {
    if (m_extensionEncoder.GetPosition() > 75 && !m_limitSwitch.Get()) 
        return true;
    else return false;
}

void ArmSubsystem::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Arm");

    builder.AddDoubleProperty("ArmPosition", LAMBDA(m_extensionEncoder.GetPosition()), nullptr);
    builder.AddIntegerProperty("ArmAngle", LAMBDA(GetAngle()), nullptr);

    builder.AddStringProperty("Actual State", LAMBDA(StateToString(GetState())), nullptr);
    builder.AddStringProperty("Target State", LAMBDA(StateToString(GetTarget())), nullptr);
}

void ArmSubsystem::CheckState() {
    double extension = m_extensionEncoder.GetPosition();
    
    // Check Stow
    if (hb::InRange(extension, m_stow.extension, EPSILON_EXTENSION) && hb::InRange(GetAngle(), m_stow.angle, EPSILON_ANGLE)) {
        m_actualState = State::kStow;
        return;
    }
    
    // Check Mid Cone
    if (hb::InRange(extension, m_coneMid.extension, EPSILON_EXTENSION) && hb::InRange(GetAngle(), m_coneMid.angle, EPSILON_ANGLE)) {
        m_actualState = State::kMidCone;
        return;
    }

    // Check Mid Cube Cone Pickup
    if (hb::InRange(extension, m_cubeMidconePickup.extension, EPSILON_EXTENSION) && hb::InRange(GetAngle(), m_cubeMidconePickup.angle, EPSILON_ANGLE)) {
        m_actualState = State::kMidCubeConePickup;
        return;
    }


    //Check Cube Pickup
    if (hb::InRange(extension, m_cubePickup.extension, EPSILON_EXTENSION) && hb::InRange(GetAngle(), m_cubePickup.angle, EPSILON_ANGLE)) {
        m_actualState = State::kCubePickup;
        return;
    }

    if (hb::InRange(extension, m_MsMaiCar.extension, EPSILON_EXTENSION) && hb::InRange(GetAngle(), m_MsMaiCar.angle, EPSILON_ANGLE)) {
        m_actualState = State::kMsMaiCar;
        return;
    }

    m_actualState = State::kRunning;
    
}

std::string ArmSubsystem::StateToString(State state) {
    switch (state) {
        case State::kCubePickup: 
            return "Cube Pickup";
        break;

        case State::kMidCone: 
            return "Mid Code";
        break;

        case State::kMidCubeConePickup: 
            return "Mid Cube or Cone Pickup";
        break;

        case State::kMsMaiCar: 
            return "Ms Mai Car";
        break;

        case State::kRunning: 
            return "Running";
        break;

        case State::kStow: 
            return "Stow";
        break;
    }
}