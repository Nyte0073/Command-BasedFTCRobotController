package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.SwerveDrive;

@TeleOp(name = "SwerveTeleop", group = "teamcode")
public class SwerveTeleop extends CommandOpMode {
    @Override
    public void initialize() {
        Motor[] turningMotors = new Motor[] {
                new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)
        };
        Motor[] drivingMotors = new Motor[] {
                new Motor(hardwareMap, Constants.SwerveConstants.frontLeftDriving),
                new Motor(hardwareMap, Constants.SwerveConstants.frontRightDriving),
                new Motor(hardwareMap, Constants.SwerveConstants.backLeftDriving),
                new Motor(hardwareMap, Constants.SwerveConstants.backRightDriving)
        };
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        SwerveDrive swerveDrive = new SwerveDrive(telemetry, turningMotors, drivingMotors, imu, gamepadEx);
        SwerveDriveCommand swerveDriveCommand = new SwerveDriveCommand(swerveDrive, gamepadEx, imu);
        swerveDrive.setDefaultCommand(swerveDriveCommand);
        register(swerveDrive);
    }
}