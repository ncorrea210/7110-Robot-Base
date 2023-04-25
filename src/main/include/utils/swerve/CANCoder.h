#pragma once 

#include <ctre/phoenix.h>
#include <units/angle.h>
#include <numbers>

namespace hb{
  class CANcode : CANCoder{
    public:
      explicit CANcode(const int& id, const double& offset);

      /**
       * @brief gets the positon of the encoder from -pi to pi
       * and applies the offset
       * 
       * @return -pi , pi (double)
       */
      double Get();

    private:

      double m_offset;

      int m_ID;

    };
} // namespace hb
