#include "utils/swerve/PigeonGyro.h"

#include <frc/Timer.h>


#include <cmath>
#include <utility>
#include <numbers>

using namespace ctre::phoenix::sensors;
using namespace hb;

static int sgn(double v) {
  return v >= 0 ? 1 : -1;
}

pigeonGyro::pigeonGyro(int ID) {
  pigeon = new ctre::phoenix::sensors::PigeonIMU(ID);
  pigeon->ConfigFactoryDefault();
  m_offset = 0;
}

double pigeonGyro::GetAngle() const {
  if (pigeon->GetState() == PigeonIMU::Ready) {
    PigeonIMU::FusionStatus stat;
    pigeon->GetFusedHeading(stat);
    m_angle = stat.heading;
  } 
  return -m_angle - m_offset;
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
  // actual - offset = angle
  // actual - angle = offset
  m_offset;
  m_offset = GetAngle() - angle.value();
}

double pigeonGyro::GetCompassHeading() const {

  double angle = GetAngle();
  bool signChange = false;
  int initSign = sgn(angle);

  if (fabs(angle) < 180) return angle;

  // add or subtract until the angle switches sign
  int i = 0;
  for (i; i < 30; i++) {
    angle -= initSign * 360;
    if (fabs(angle) <= 180) {
      break;
    }
    // signChange = sgn(angle) != initSign ? true : false;
    // if (signChange) {
    //   break;
    // }
  }

  // When sign change is true, add back the value that was last removed to get the true heading
  // angle += initSign * 360;

  return angle;

}