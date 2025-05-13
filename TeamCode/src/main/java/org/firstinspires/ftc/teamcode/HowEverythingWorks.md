# How Everything Works in This Swerve Drivetrain FTC Repository

## 1. File Structure 

##### This FTC repository is programmatically structured to use FTC LIB, a form of command-based programming libraries that help people working in FTC implement the same command-based used in FRC programming in Android Studio. The structure of command based programming is simple - and here is a full break down of command-based programming for beginners:

## Subsystems

##### Subsystems are the simplest level of command-based programming, as they include code that operates each individual piece of hardware on the robot. For example, one of the subsystems in your robot programming repo could be named 'Drivetrain', because inside that class, it contains code to spin the robot's wheels in certain directions and in certain configurations to drive the robot around. To create a subsystem with FTC LIB, all you have to is make your class extend the `SubsystemBase` class that is provided by FTC LIB itself. This will turn your class into a subsystem, and will also give you access to override the `periodic()` method that is provided by the `SubsystemBase` class as way for you to send any data about the subsystem you want to the robot's `Telemetry` system to display on screen.

## Commands

##### Commands are the next level above subsystems in the FTC LIB chain of command-based programming. Essentially what commands are is that they are classes that have control over what certain subsystems do at specific, depending on if the robot needs a certain subsystem or certain part of a subsystem to activate or not. In simple terms, commands are classes that control when certain subsystems are certain parts of subsystems activate to make the robot perform specific actions.
