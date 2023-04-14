#include "utils/subsystems/LEDSubsystem.h"

using namespace hb;

LEDSubsystem::LEDSubsystem(const unsigned port, const unsigned length) : frc::AddressableLED(port), m_length(length) {
  m_data = new frc::AddressableLED::LEDData[length];
  SetLength(length);
  SetData({*m_data});
  Start();
}

void LEDSubsystem::SetRGB(unsigned R, unsigned G, unsigned B) {
  for (unsigned i = 0; i < m_length; i++) {
    m_data[i].SetRGB(R, G, B);
  }
  SetData({*m_data});
}
