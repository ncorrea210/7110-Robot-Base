#include "utils/PigeonGyro.h"

#include <cmath>
#include <utility>
#include <frc/Timer.h>
#include <numbers>

using namespace ctre::phoenix::sensors;
using namespace hb;

pigeonGyro::pigeonGyro(int ID) {
  pigeon = new ctre::phoenix::sensors::PigeonIMU(ID);
  pigeon->ConfigFactoryDefault();
}

double pigeonGyro::GetAngle() const {
  if (pigeon->GetState() == PigeonIMU::Ready) {
    PigeonIMU::FusionStatus stat;
    pigeon->GetFusedHeading(stat);
    m_angle = stat.heading;
  } 
  return -m_angle;
}

double pigeonGyro::GetRate() const {
  if (pigeon->GetState() == PigeonIMU::Ready) {
    double rate[3];
    pigeon->GetRawGyro(rate);
    m_rate = rate[2];
  } 
  return m_rate;
}

void pigeonGyro::Reset() {
  pigeon->SetFusedHeading(0, 30);
  m_angle = m_rate = 0;
}

double pigeonGyro::GetPitch() {
  return pigeon->GetPitch();
}

double pigeonGyro::GetRoll() {
  return pigeon->GetRoll();
}

void pigeonGyro::Calibrate() {} // Gyro::Calibrate() is pure virtual

frc::Rotation2d pigeonGyro::GetRot2d() {
  return frc::Rotation2d(units::degree_t(GetAngle()));
}

units::radian_t pigeonGyro::GetRad() const {
  return units::radian_t((std::numbers::pi * GetAngle()) / 180);
}

void pigeonGyro::SetPosition(units::degree_t angle) {
  pigeon->SetFusedHeading(angle.value(), 30);
}