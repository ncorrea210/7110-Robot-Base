#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>


namespace hb {

  class LEDSubsystem : public frc2::SubsystemBase, public frc::AddressableLED {
    
    public:

      virtual void SetRGB(unsigned R, unsigned G, unsigned B);

    protected:

      explicit LEDSubsystem(const unsigned port, const unsigned length);
    
    private:

      const unsigned m_length;

      frc::AddressableLED::LEDData* m_data;
      
  };

}