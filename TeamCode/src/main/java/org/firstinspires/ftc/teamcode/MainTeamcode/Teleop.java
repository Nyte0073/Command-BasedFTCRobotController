package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Commands.ClawCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "Teleop", group = "teamcode")
public class Teleop extends CommandOpMode {
    Motor[] motors;
    private DriveCommand driveCommand;
    private ClawCommand clawCommand;
    private Drivetrain drivetrain;
    private ServoEx clawServo;
    private Claw claw;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            run();
        }
    }

    @Override
    public void initialize() {
        final GamepadEx gamepadEx = new GamepadEx(gamepad1);

        final IMU imu = hardwareMap.get(IMU.class, "imu");

        final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        sleep(500);

        motors = new Motor[]{new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)};

        clawServo = new SimpleServo(hardwareMap, Constants.ServoConstants.clawServo, 0, 360);

        drivetrain = new Drivetrain(telemetry, motors, imu, gamepadEx);
        driveCommand = new DriveCommand(motors, telemetry, imu, gamepadEx, true, drivetrain);
        claw = new Claw(clawServo, telemetry);
        clawCommand = new ClawCommand(claw, gamepadEx);

        drivetrain.setDefaultCommand(driveCommand);
        claw.setDefaultCommand(clawCommand);
    }
}
