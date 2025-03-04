package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Commands.ButtonCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.TriggerCommand;
import org.firstinspires.ftc.teamcode.Otos_Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Teleop", group = "teamcode")
public class Teleop extends CommandOpMode {
    Motor[] motors;
    private Drivetrain drivetrain;
    MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
   static IMU imu;
   static GamepadEx gamepadEx;
   TriggerReader leftReader, rightReader;

   List<GamepadKeys.Button> buttons = List.of(
           GamepadKeys.Button.A,
           GamepadKeys.Button.B,
           GamepadKeys.Button.X,
           GamepadKeys.Button.Y,
           GamepadKeys.Button.DPAD_DOWN,
           GamepadKeys.Button.DPAD_UP,
           GamepadKeys.Button.DPAD_LEFT,
           GamepadKeys.Button.DPAD_RIGHT
   );

    @Override
    public void initialize() {
        gamepadEx = new GamepadEx(gamepad1);

         imu = hardwareMap.get(IMU.class, "imu");

        final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        sleep(500);

        motors = new Motor[]{new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
                new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)};

        drivetrain = new Drivetrain(telemetry, motors, imu, gamepadEx);
        DriveCommand driveCommand = new DriveCommand(motors, telemetry, imu, gamepadEx, true, drivetrain);

        leftReader = new TriggerReader(gamepadEx, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightReader = new TriggerReader(gamepadEx, GamepadKeys.Trigger.RIGHT_TRIGGER);

        register(drivetrain);

        drivetrain.setDefaultCommand(driveCommand);

        imu.resetYaw();
    }

    @Override
    public void run() {
        gamepadEx.readButtons();

        mecanumDrive.updatePoseEstimate();
        telemetry.addData("Robot Position X", mecanumDrive.pose.position.x);
        telemetry.addData("Robot Position Y", mecanumDrive.pose.position.y);

        if(leftReader.wasJustPressed()) {
            TriggerCommand.leftTriggerPressed = true;
            schedule(new TriggerCommand(gamepadEx, telemetry));
        } else if(rightReader.wasJustPressed()) {
            TriggerCommand.rightTriggerPressed = true;
            schedule(new TriggerCommand(gamepadEx, telemetry));
        }

        for(GamepadKeys.Button button : buttons) {
            if(gamepadEx.wasJustPressed(button)) {
                schedule(new ButtonCommand(gamepadEx, button, telemetry));
            }
        }

        CommandScheduler.getInstance().run();
    }
}
