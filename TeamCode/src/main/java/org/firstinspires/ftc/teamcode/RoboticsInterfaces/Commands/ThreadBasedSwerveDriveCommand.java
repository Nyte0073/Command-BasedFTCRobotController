package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.ThreadBasedSwerveDrive;

public class ThreadBasedSwerveDriveCommand extends CommandBase {

    /**Reference the to the {@code ThreadBasedSwerveDrive} object that is being used to connect the swerve drive software
     * to all the robot's swerve hardware components.*/
    ThreadBasedSwerveDrive swerveDrive;

    /**Reference to the human driver's Xbox controller.*/
    GamepadEx gamepadEx;

    /**Reference to the orientation system of the robot. This system returns the current heading that robot is facing in degrees.*/
    IMU imu;

    /**Constructs a new {@code ThreadBasedSwerveDriveCommand()} with an initialized {@code ThreadBasedSwerveDrive},
     * {@code GamepadEx} and {@code IMU}.*/
    public ThreadBasedSwerveDriveCommand(ThreadBasedSwerveDrive swerveDrive, GamepadEx gamepadEx, IMU imu) {
        this.swerveDrive = swerveDrive;
        this.gamepadEx = gamepadEx;
        this.imu = imu;
    }

    /**When this code executes, depending on what button is pressed, it will either make the robot reset its IMU
     * or stop the entirely (in the case of an emergency).*/
    @Override
    public void execute() {
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            swerveDrive.resetGyro(imu);
        } else if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            swerveDrive.stopMotors();
            swerveDrive.stopThreads();
        } else {
           try {
               swerveDrive.drive();
           } catch(Exception e) {
               throw new RuntimeException(e);
           }
        }
    }
}
