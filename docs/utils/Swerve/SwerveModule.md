[Back](/docs/utils/Swerve/Swerve.md)

### [SwerveModule.h](/src/main/include/utils/swerve/SwerveModule.h) [SwerveModule.cpp](/src/main/cpp/utils/swerve/SwerveModule.cpp)

## Methods:

### SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderID, double offset)
* driveMotorID: CAN ID of the driving Neo Motor
* turnMotorID: CAN ID of the turning Neo Motor
* turnEncoderID: CAN ID of the CANCoder
* offset: The offset in degrees needed to have the module face forwards

### frc::SwerveModuleState GetState()
Returns a SwerveModuleState object that contains the velocity and angle of a module. (velocity in M/S and angle in radians)

### frc::SwerveModulePosition GetPosition()
Returns a SwerveModulePosition object that contains the number of meters traveled by the wheel and the angle of the wheel (angle in radians)

### void SetDesiredState(frc::SwerveModuleState)
Takes in the requested velocity and angle of the module, then calculates and applies the necessary outputs on the two Neo motors to get to that desired state. 

### void ResetEncoders() 
Resets the number of meters traveled by the drive motor as well as re-zeroing the angle motor. 

### void ZeroTurnEncoder()
Re-zeros the turn motor with the CANCoder

### void StopMotors()
Stops all outputs to the motors immediately

## Devices:
Per module:
- 2 Neo Motors
- 1 CANCoder
