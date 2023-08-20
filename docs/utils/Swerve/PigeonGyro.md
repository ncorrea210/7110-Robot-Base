[Back](/docs/utils/Swerve/Swerve.md)

### [PigeonGyro.h](/src/main/include/utils/swerve/PigeonGyro.h) [PigeonGyro.cpp](/src/main/cpp/utils/swerve/PigeonGyro.cpp)

## Methods: 

### PigeonGyro(int ID)
* ID - The CAN ID of the pigeon gyro

### double GetAngle()
Returns the cummulative angle of the gyro. This does not wrap arround, it will continue past -180 and +180. For example it could read 720 which would be the same as 0 

### double GetRate()
Returns the rate the gyro is rotation in degrees per second

### void Reset()
Resets the heading of the pigeon gyro to zero

### void Calibrate()
*** This function doesn't do anything, it just satisfies a unnecessary pure virtual function***

### double GetPitch()
Returns the pitch of the gyro in degrees

### double GetRoll()
Returns the roll of the gyro in degrees

### frc::Rotation2d GetRot2d()
Returns a Rotation2d object which is necessary for methods given by WPI. It is simply an angle

### units::radian_t GetRad()
Returns the angle of the gyro in radians

### void SetPosition(units::degree_t)
Sets the angle of the gyro to what is specified in the arguments

### double GetCompassHeading()
Returns the heading of the robot bounded to [-180,180]