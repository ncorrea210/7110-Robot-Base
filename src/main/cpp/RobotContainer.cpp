// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "RobotContainer.h"

#include <utility>
#include <cmath>
#include <numbers>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/ConditionalCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/cams/Limelight.h"


using namespace DriveConstants;


RobotContainer::RobotContainer() {

  /**
   * Adding options to chooser should be done as such
   * m_chooser.AddOption("Auto Name", &Auto);
   * Where auto has already been declared in RobotContainer.h
  */

  // frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
  // frc::SmartDashboard::PutData("Swerve", &m_drive);
  frc::SmartDashboard::PutData("Arm", &m_arm);

  // Configure the button bindings
  ConfigureButtonBindings();

  //Default subsystem commands are defined in this section
  m_drive.SetDefaultCommand(DefaultDriveCMD(&m_drive, 
        [this] {return (m_speedLimitx.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY() < 0 ? -(m_driverController.GetLeftY() * m_driverController.GetLeftY()) : (m_driverController.GetLeftY() * m_driverController.GetLeftY()), 0.01)));}, 
        [this] {return -m_speedLimity.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX() < 0 ? -(m_driverController.GetLeftX() * m_driverController.GetLeftX()) : (m_driverController.GetLeftX() * m_driverController.GetLeftX()), 0.01));},
        [this] {return -(frc::ApplyDeadband(m_driverController.GetRightX() < 0 ? -(m_driverController.GetRightX() * m_driverController.GetRightX()) : (m_driverController.GetRightX() * m_driverController.GetRightX()), 0.025) * (double)DriveConstants::kMaxAngularSpeed);}, 
        [this] {return true;},
        [this] {return m_drive.GetSpeed().value();}));

}

void RobotContainer::ConfigureButtonBindings() {
  /**
   *  When binding buttons use this as the template
   *  frc::JoystickButton(&m_controller, frc::XboxController::Button::kButton).WhenPressed(Command);
   *  TODO: test command xbox controller implementation(found below) and see if it works
  */
  // m_driver.A().WhenActive(frc2::InstantCommand([this] {m_drive.ZeroHeading();}));
  // m_driverController.A().WhenActive(frc2::InstantCommand([this] {hb::limeLight::SetLED(hb::limeLight::LEDMode::kOff);}));
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA).WhenPressed(frc2::InstantCommand([this] {hb::limeLight::SetLED(hb::limeLight::LEDMode::kOff);}));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY).WhenPressed([this] {m_arm.RunExtension(0.1);})
    .WhenReleased([this] {m_arm.RunExtension(0);});
  
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA).WhenPressed([this] {m_arm.RunExtension(-0.1);})
    .WhenReleased([this] {m_arm.RunExtension(0);});

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB).WhenPressed([this] {m_arm.RunActuator(0.5);})
    .WhenReleased([this] {m_arm.RunActuator(0);});

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).WhenPressed([this] {m_arm.RunActuator(-0.5);})
    .WhenReleased([this] {m_arm.RunActuator(0);});
  
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper).WhenPressed([this] {m_arm.Stow();});

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhenPressed([this] {m_arm.MidCone();});

}


frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}
