package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.MecanumDrive;

@TeleOp(name = "MecanumTeleop", group = "teamcode")
public class MecanumTeleop extends CommandOpMode {
    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        Motor[] motors = new Motor[] {
                new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)
        };
        IMU imu = hardwareMap.get(IMU.class, "imu");
        gamepadEx = new GamepadEx(gamepad1);
        MecanumDrive mecanumDrive = new MecanumDrive(motors, telemetry, imu, true, gamepadEx);
        MecanumDriveCommand mecanumDriveCommand = new MecanumDriveCommand(mecanumDrive, gamepadEx, imu);
        mecanumDrive.setDefaultCommand(mecanumDriveCommand);
    }
}
