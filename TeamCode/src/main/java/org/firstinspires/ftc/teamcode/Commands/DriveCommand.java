package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class DriveCommand extends CommandBase {
    Drivetrain drivetrain;
    boolean fieldOriented;
    GamepadEx gamepadEx;
    Motor[] motors;
    Telemetry telemetry;
    IMU imu;
    public DriveCommand(Motor[] motors, Telemetry telemetry, IMU imu, GamepadEx gamepadEx, boolean fieldOriented, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.fieldOriented = fieldOriented;
        this.gamepadEx = gamepadEx;
        this.motors = motors;
        this.telemetry = telemetry;
        this.imu = imu;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(fieldOriented);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
