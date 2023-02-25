#include "utils/CANCoder.h"

using namespace hb;

CANcode::CANcode(const int& Id, const double& offset = 0) : CANCoder(Id), m_offset(offset), m_ID(Id) {
  printf("CANCoder: %i, reading %5.2f\n", Id, GetAbsolutePosition() - 180);
}

double CANcode::Get() {
   // double calc = ((GetAbsolutePosition() - m_offset)/360.0) * 2.0 * std::numbers::pi - std::numbers::pi;
  // if(m_ID == 0) printf("%d: %5.2f\n", m_ID, calc);
  // return calc; 
  // 
  double rv = GetAbsolutePosition() - m_offset;

  // rv -= 180.0;
    
  if (rv < 0.0) rv += 360.0;

  if (rv > 360.0) rv -= 360.0;

  double a = (std::numbers::pi * 2 * (rv / 360.0)) - (std::numbers::pi);

    // if (m_ID == 2) printf("%d: %5.2f\n", m_ID, a);

  return -a;
}