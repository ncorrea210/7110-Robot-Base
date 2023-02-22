#pragma once

#include <frc/interfaces/Gyro.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

namespace hb {

  class pigeonGyro : public frc::Gyro {
  public:

    /**
     * @brief Creates a new pigeon gyro object using CAN 
     * 
     * @param ID the CAN ID
     */
    explicit pigeonGyro(int ID);

    /**
     * @brief Gets the angle the pigeon gyro is reading
     * 
     * @return angle in degrees 
     */
    double GetAngle() const override;

    /**
     * @brief Gets the rate of rotation in degrees per second
     * 
     * @return double rate of rotation 
     */
    double GetRate() const override;

    /**
     * @brief Resets the pigeon gyro heading
     */
    void Reset() override;

    /**
     * @brief satisfies the pure virtual Calibrate() function found in
     * frc::Gyro
     */
    void Calibrate() override;

  private:
    ctre::phoenix::sensors::PigeonIMU* pigeon;
    mutable double m_angle;
    mutable double m_rate;
  };
} // namespace hb
