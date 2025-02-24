package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "Teleop", group = "teamcode")
public class Teleop extends CommandOpMode {
    Motor[] motors;
    private final IMU imu = hardwareMap.get(IMU.class, "imu");
    final GamepadEx gamepadEx = new GamepadEx(gamepad1);
    private DriveCommand driveCommand;
    private ClawCommand clawCommand;
    private Drivetrain drivetrain;
    private ServoEx clawServo;
    private Claw claw;

    @Override
    public void initialize() {
        clawServo = new SimpleServo(hardwareMap, Constants.ServoConstants.clawServo, 0, 360);
        claw = new Claw(clawServo, telemetry);
       driveCommand = new DriveCommand(motors, telemetry, imu, gamepadEx, true);
       drivetrain = new Drivetrain(telemetry, motors, imu, gamepadEx);
       clawCommand  = new ClawCommand(claw, gamepadEx);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        motors = new Motor[]{new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)};

        drivetrain.setDefaultCommand(driveCommand);
        claw.setDefaultCommand(clawCommand);
    }
}
