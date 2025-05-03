package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class DriveCommand extends CommandBase {

    /**The robot's drivetrain.*/
    Drivetrain drivetrain;

    /**Boolean state of whether the robot wants to be driven field oriented or not.*/
    boolean fieldOriented;

    /**The driver's controller.*/
    GamepadEx gamepadEx;

    /**The drivetrain's motors.*/
    Motor[] motors;

    /**The robot's gyroscope system.*/
    IMU imu;

    public static DriveStates driveState = DriveStates.DISABLE_SLOW_MODE;

    /**Constructs a new {@code DriveCommand()} with initialized {@code Motor}'s, {@code Telemetry}, {@code IMU} and {@code GamepadEx.} */
    public DriveCommand(Motor[] motors, IMU imu, GamepadEx gamepadEx, boolean fieldOriented, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.fieldOriented = fieldOriented;
        this.gamepadEx = gamepadEx;
        this.motors = motors;
        this.imu = imu;

        addRequirements(drivetrain); //This command requires the Drivetrain subsystem.
    }

    @Override
    public void execute() {
        switch(driveState) {
            case ENABLE_SLOW_MODE:
                drivetrain.mecanumDrive.setMaxSpeed(0.5);
                break;

            case DISABLE_SLOW_MODE:
                drivetrain.mecanumDrive.setMaxSpeed(1);
                break;
        }
        drivetrain.drive(fieldOriented);
    }

    public enum DriveStates {
        ENABLE_SLOW_MODE,
        DISABLE_SLOW_MODE
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
