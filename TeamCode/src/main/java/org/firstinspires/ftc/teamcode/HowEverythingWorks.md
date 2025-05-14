# How Everything Works in This Swerve Drivetrain FTC Repository

## 1. File Structure 

##### This FTC repository is programmatically structured to use FTC LIB, a form of command-based programming libraries that help people working in FTC implement the same command-based used in FRC programming in Android Studio. The structure of command based programming is simple - and here is a full break down of command-based programming for beginners:

## Subsystems

##### Subsystems are the simplest level of command-based programming, as they include code that operates each individual piece of hardware on the robot. For example, one of the subsystems in your robot programming repo could be named 'Drivetrain', because inside that class, it contains code to spin the robot's wheels in certain directions and in certain configurations to drive the robot around. To create a subsystem with FTC LIB, all you have to do is make your class extend the `SubsystemBase` class that is provided by FTC LIB itself. This will turn your class into a subsystem, and will also give you access to override the `periodic()` method that is provided by the `SubsystemBase` class as way for you to send any data about the subsystem you want to the robot's `Telemetry` system to display on screen.

## Commands

##### Commands are the next level above subsystems in the FTC LIB chain of command-based programming. Essentially what commands are is that they are classes that have control over what certain subsystems do at specific times or in specific situations, depending on if the robot needs a certain subsystem or certain part of a subsystem to activate or not. In simple terms, commands are classes that control when certain subsystems are certain parts of subsystems activate to make the robot perform specific actions. For example, going back to our 'Drivetrain' subsystem we were talking about earlier, you could also have a command named 'DriveCommand' that controls whether the 'Drivetrain' subsystem makes the robot drive in certain directions or drive at all depending on the controller input from the human driver. To create a command with FTC LIB, all you have to do is make your class extend the `CommandBase` class that is provided by FTC LIB. This will turn your class into a command and also will give you access to all the command methods (also provided by FTC LIB) to make your command make its corresponding subsystem do certain things. There are 4 methods you can override when you extend the `CommandBase` class, and those methods are `initialize()` for initializing variables within the class, `execute()` which is the method you put the code you want the command to actually run when the command itself is scheduled to run, `isFinished()` in which you will provide a boolean state telling the command at state does the robot have to be in for the command to be able to finish, and `end()`, the method where you can provide code for the command to run when the command is finishing up because the `isFinished()` method returned true.

## Teleop

##### Once you have created your necessary Subsystems and Commands for your robot, you will then create instance of those classes in what we call a 'Teleop' class, which is the class that controls all the commands and thus their corresponding subsystems.
