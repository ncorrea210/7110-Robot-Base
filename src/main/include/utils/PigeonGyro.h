#pragma once

#include <frc/interfaces/Gyro.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <units/angle.h>
#include <frc/Timer.h>

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

    double GetPitch();

    double GetRoll();

    /**
     * Working version of GetRotation2d
    */
    frc::Rotation2d GetRot2d();

    double GetIntDist();

    double Get2IntDist();

    units::radian_t GetRad() const;

    void SetPosition(units::degree_t);

  private:
    ctre::phoenix::sensors::PigeonIMU* pigeon;
    mutable double m_angle;
    mutable double m_rate;
    double m_lastTime;
    frc::Timer m_timer;
    double m_dist;
  };
} // namespace hb
