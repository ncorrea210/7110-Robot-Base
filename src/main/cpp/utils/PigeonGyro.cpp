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
  m_lastTime = 0;
  m_dist = 0;
  m_timer.Start();
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
  m_angle = m_rate = m_dist = 0;
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

double pigeonGyro::GetIntDist() {
  double calc = (GetRate() * (m_timer.Get().value() - m_lastTime));
  m_dist = calc + m_dist;
  m_lastTime = m_timer.Get().value();
  return calc + m_dist;
}

units::radian_t pigeonGyro::GetRad() const {
  return units::radian_t((std::numbers::pi * GetAngle()) / 180);
}