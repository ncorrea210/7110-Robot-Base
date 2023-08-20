[Back](/docs/utils/Swerve/Swerve.md)

### [CANCoder.h](/src/main/include/utils/swerve/CANCoder.h) [CANCoder.cpp](/src/main/cpp/utils/swerve/CANCoder.cpp)

## Methods:

### CANCoder(int id, double offset)
* id: CAN ID of the CANCoder
* offset: an offset to the reading

### double Get()
Returns the angle of the CANCoder bounded from [-pi,pi]

## Devices: 
* CANCoder