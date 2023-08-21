[Back](/README.md)

# Robot Overview

## Robot Container
### [RobotContainer.h](/src/main/include/RobotContainer.h) [RobotContainer.cpp](/src/main/cpp/RobotContainer.cpp)

The robot container essentially "commands" the robot. This file is where all the subsystems come together and where the driver controls the actual subsystems on the robot. The robot continer does these things in particular:
* Binds controls from the drivers to the robot
* Places all information onto the dashboard
* Contains the autonomous to be run by the robot
Although this file is important, it really isn't a file that is accessed very often. All you really need to do here is add autonomous commands to the dashboard and rebind controls for the drivers. 

## Robot
### [Robot.h](/src/main/include/Robot.h) [Robot.cpp](/src/main/cpp/Robot.cpp)

Much like the robot container, this file really isn't something to look at all the time. This file essentially just starts command based and hands off control to the command scheduler. 

## Command Based Overview

Command based programming takes everything that timed robot does, and breaks it down into it's individual parts. Here's a quick breakdown:  
    Subsystem:  
    A subsystem is a physical system on the robot. For example, the arm might be a subsystem. The point of having a subsystem is in order to organize methods that are related to the subsytem so it can be coherently controlled. Although a subsystem contains all the functions that control the device, it cannot actually be controlled without a command that controls it.  
    Command:  
    A command is something that runs any number of subsystems. The point of a command is to have the subsystem do one very specific task and then stop when the task is done. It can also orchestrate several subsystems together to do complex tasks.  
A big reason that this is done is to simplify working on files. keeping everything seperate makes it easier to collaborate and makes it easier to know where you are in the file. 

## More info
- [Subsystems](/docs/subsystems/Subsystems.md)
- [Commands](/docs/commands/Commands.md)
- [Utils](/docs/utils/Util.md)
