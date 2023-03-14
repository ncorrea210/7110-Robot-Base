#include "utils/NeoMotors.h"

using namespace hb;

NeoMotor::NeoMotor(const int& Id, rev::CANSparkMax::MotorType type, rev::CANSparkMax::IdleMode mode) : 
rev::CANSparkMax(Id, type), rev::SparkMaxRelativeEncoder(GetEncoder()){
  if (rev::CANSparkMax::GetIdleMode() != mode){
  rev::CANSparkMax::SetIdleMode(mode);
  BurnFlash();
  }
}

void NeoMotor::SetRPM2MPS(const double& Ratio) {
  m_Ratio = Ratio;
}

double NeoMotor::GetRate() const {
  return (GetVelocity()/60) * m_Ratio;
}

void NeoMotor::SetRPMpid(double& RPM) {
  Set(m_PID.Calculate(GetVelocity(), RPM));
}

double NeoMotor::GetDistance() {
  double dist = GetPosition() * m_Ratio;
  return dist;
}