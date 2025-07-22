package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands.HolonomicDriveCommand;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.HolonomicDrive;

@TeleOp(name = "HolonomicTeleop", group = "teamcode")
public class HolonomicTeleop extends CommandOpMode {

    @Override
    public void initialize() {

        /*Initializing everything for the subsystems and commands.*/
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        IMU imu = hardwareMap.get(IMU.class, "imu");

        Motor[] HDriveMotors = new Motor[] {
              new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor)
        },

        XDriveMotors = new Motor[] {
                new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)
        };

        HolonomicDrive holonomicDrive = new HolonomicDrive("HDrive", HDriveMotors, XDriveMotors, gamepadEx,
                Constants.HolonomicConstants.motorAngles, imu, true);

        HolonomicDriveCommand driveCommand = new HolonomicDriveCommand(holonomicDrive, imu, gamepadEx);
        holonomicDrive.setDefaultCommand(driveCommand);

        /*Holonomic drive subsystem needs this, or else its 'periodic' method won't be called.*/
        register(holonomicDrive);
    }
}
