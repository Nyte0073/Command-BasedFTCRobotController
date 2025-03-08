package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    /**The robot's telemetry system.*/
    Telemetry telemetry;

    /**The robot's gyroscope system.*/
    IMU imu;

    /**Constructs a new {@code DriveCommand()} with initialized {@code Motor}'s, {@code Telemetry}, {@code IMU} and {@code GamepadEx.} */
    public DriveCommand(Motor[] motors, Telemetry telemetry, IMU imu, GamepadEx gamepadEx, boolean fieldOriented, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.fieldOriented = fieldOriented;
        this.gamepadEx = gamepadEx;
        this.motors = motors;
        this.telemetry = telemetry;
        this.imu = imu;

        addRequirements(drivetrain); //This command requires the Drivetrain subsystem.
    }

    @Override
    public void execute() {
        if(drivetrain.getGamepadEx().wasJustPressed(GamepadKeys.Button.A)) { //If button A was pressed, cut the max speed of the robot in half.
            drivetrain.mecanumDrive.setMaxSpeed(0.5);
        }

        if(drivetrain.getGamepadEx().wasJustPressed(GamepadKeys.Button.B)) { //If button B was pressed, increase the max speed of the robot to 1.
            drivetrain.mecanumDrive.setMaxSpeed(1);
        }

        drivetrain.drive(fieldOriented); //Make the drive either field oriented or not.
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
