package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.SwerveDrive;

public class SwerveDriveCommand extends CommandBase {
    /**Reference to the robot swerve drivetrain - access to all four wheels and to methods that can control the
     * wheels to perform different depending on the user input on what the robot should do.*/
    SwerveDrive swerveDrive;

    /**Reference to the driver's gamepad (Xbox) controller.*/
    GamepadEx gamepadEx;

    /**The IMU orientation system of the robot that returns the exact angle (in degrees or in radians) that the robot is currently
     * facing relative to an orientation that is considered 0 degrees by the IMU.*/
    IMU imu;

    /**Constructs a new {@code SwerveDriveCommand()} with an initialized {@code SwerveDrive}, {@code GamepadEx},
     * and {@code IMU}.*/
    public SwerveDriveCommand(SwerveDrive swerveDrive, GamepadEx gamepadEx, IMU imu) {
        this.swerveDrive = swerveDrive;
        this.gamepadEx = gamepadEx;
        this.imu = imu;
    }

    /**When this method runs, the {@code SwerveDriveCommand} class will either make the robot stop all of its motors
     * (if B is pressed on the Xbox controller), reset the yaw of its IMU system (if A was pressed on the Xbox controller), or drive using
     * its swerve drivetrain (if none of the above were pressed).*/
    @Override
    public void execute() {
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            swerveDrive.resetGyro(imu);
        } else if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            swerveDrive.stop();
        } else {
            swerveDrive.drive();
        }
    }
}
