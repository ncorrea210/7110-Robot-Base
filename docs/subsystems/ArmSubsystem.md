[Back](/docs/subsystems/Subsystems.md)

### [ArmSubsystem.h](/src/main/include/subsystems/ArmSubsystem.h) [ArmSubsystem.cpp](/src/main/cpp/subsystems/ArmSubsystem.cpp)

## Methods:

### struct ArmPosition 
- uint16_t extension
- uint16_t angle
This is a struct that contains the extension and angle

### ArmSubsystem()
The contructor does not take in any arguements. All initializing values come from Constants.h in the ArmConstants namespace

### enum class State
Contains all the arm positions the arm may be in. 
- kStow
- kMidCone
- kMidCubeConePickup
- kCubePickup
- kMsMaiCar
- kRunning

### void Periodic()
The periodic function contiains anything that should be called constantly at 50hz for the subsystem to run. In the case of this subsystem, it updates the dashboard with the current and target state. It also contains the PID loops that run the arm. 

### void StopMotors()
Stops the motors of the arm instantly and for the duration that the function is called.

### void SetPosition(ArmPostion)
Sets the position of the arm based on the arm positon passed into it .

### void Stow()
Sets the arm to the stow position.

### void MidCone()
Sets the position of the arm in the mid cone scoring position.

### void MidCubeConePickup()
Sets the position of the arm to the mid cube scoring position and the cone pickup position (They are the same position).

### void CubePickup()
Sets the position of the arm in the cube pickup position.

### void MsMaiCar()
Sets the arm in a spot where the robot can be loaded into Ms. Mai's car. This is not a joke, this is a serious function. It makes life easier and it will be kept. 

### State GetState()
Returns the state of the arm from the list of states in the state enum.

### State GetTarget()
Returns the target state of the arm from the list of states in the state enum.

### int GetAngle()
Returns the angle of the arm from [0,100]. (No unit associated)

### bool SwitchLow()
Returns true if the arm is at the low position.

### bool SwitchHigh()
Returns true if the ar is in the high position.

### void InitSendable(wpi::SendableBuilder)
Inits the smartdashboard sendable in the RobotContainer class

## Devices
- 1 Neo 550
- 1 REV Mag Limit Switch
- 1 Cim Motor
- 1 Actuator Pot