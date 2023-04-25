#pragma once

#include <rev/CANSparkMax.h>

struct SwerveConfig {
  SwerveConfig(
    int DriveMotorID,
    int TurnMotorID,
    int TurnEncoderID,
    double DriveVelocityRatio,
    double DrivePositionRatio,
    rev::CANSparkMax::IdleMode DriveIdle,
    rev::CANSparkMax::IdleMode TurnIdle,
    double DriveCurrentLimit,
    double TurnCurrentLimit,
    double DriveP,
    double DriveI,
    double DriveD,
    double DriveFF,
    double TurnP,
    double TurnI,
    double TurnD,
    double TurnFF,
    double TurnOffset    
      ) {
    this->DriveMotorID = DriveMotorID;
    this->TurnMotorID = TurnMotorID;
    this->TurnEncoderID = TurnEncoderID;
    this->DriveVelocityRatio = DriveVelocityRatio;
    this->DrivePositionRatio = DrivePositionRatio;
    this->DriveIdle = DriveIdle;
    this->TurnIdle = TurnIdle;
    this->DriveCurrentLimit = DriveCurrentLimit;
    this->TurnCurrentLimit = TurnCurrentLimit;
    this->DriveP = DriveP;
    this->DriveI = DriveI;
    this->DriveD = DriveD;
    this->DriveFF = DriveFF;
    this->TurnP = TurnP; 
    this->TurnI = TurnI;
    this->TurnD = TurnD;
    this->TurnFF = TurnFF;
    this->TurnOffset = TurnOffset;
      }

  SwerveConfig() {}

  int DriveMotorID;
  int TurnMotorID;
  int TurnEncoderID;
  double DriveVelocityRatio;
  double DrivePositionRatio;
  rev::CANSparkMax::IdleMode DriveIdle;
  rev::CANSparkMax::IdleMode TurnIdle;
  double DriveCurrentLimit;
  double TurnCurrentLimit;
  double DriveP;
  double DriveI;
  double DriveD;
  double DriveFF;
  double TurnP;
  double TurnI;
  double TurnD;
  double TurnFF;
  double TurnOffset;
};