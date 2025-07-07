package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands.ThreadBasedSwerveDriveCommand;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.ThreadBasedSwerveDrive;

@TeleOp(name = "ThreadBasedSwerveTeleop", group = "teamcode")
public class ThreadBasedSwerveTeleop extends CommandOpMode {

    @Override
    public void initialize() {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        IMU imu = hardwareMap.get(IMU.class, "imu");

        Motor[] turningMotors = new Motor[] {
                new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)
        }, drivingMotors = new Motor[] {
                new Motor(hardwareMap, Constants.SwerveConstants.frontLeftDriving),
                new Motor(hardwareMap, Constants.SwerveConstants.frontRightDriving),
                new Motor(hardwareMap, Constants.SwerveConstants.backLeftDriving),
                new Motor(hardwareMap, Constants.SwerveConstants.backRightDriving)
        };

        ThreadBasedSwerveDrive swerveDrive = new ThreadBasedSwerveDrive(telemetry, turningMotors,
                drivingMotors, imu, gamepadEx, true);

        ThreadBasedSwerveDriveCommand command = new ThreadBasedSwerveDriveCommand(swerveDrive, gamepadEx, imu);

        swerveDrive.setDefaultCommand(command);
        register(swerveDrive);
    }
}
