package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands.DifferentialDriveCommand;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.DifferentialDrive;

@TeleOp(name = "DifferentialTeleop", group = "teamcode")
public class DifferentialTeleop extends CommandOpMode {

    @Override
    public void initialize() {

        /*Initializing everything for the subsystems and commands for DifferentialDrive.*/
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        IMU imu = hardwareMap.get(IMU.class, "imu");

        Motor[] motors = new Motor[] {
            new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
            new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
            new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
            new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)
        };

        DifferentialDrive differentialDrive = new DifferentialDrive(gamepadEx, motors, "tankDrive");
        DifferentialDriveCommand differentialDriveCommand = new DifferentialDriveCommand(differentialDrive, imu, gamepadEx);
        differentialDrive.setDefaultCommand(differentialDriveCommand);

        /*Subsystem needs this method called or else it's 'periodic' method won't be run.*/
        register(differentialDrive);
    }
}
