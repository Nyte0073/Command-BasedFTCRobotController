package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Otos_Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name = "Teleop", group = "teamcode")
public class Teleop extends CommandOpMode {
    Motor[] motors;
    private Drivetrain drivetrain;
    MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
   static IMU imu;

    @Override
    public void initialize() {
        final GamepadEx gamepadEx = new GamepadEx(gamepad1);

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

        register(drivetrain);

        drivetrain.setDefaultCommand(driveCommand);

        imu.resetYaw();
    }

    @Override
    public void run() {
        mecanumDrive.updatePoseEstimate();
        telemetry.addData("Robot Position X", mecanumDrive.pose.position.x);
        telemetry.addData("Robot Position Y", mecanumDrive.pose.position.y);
        drivetrain.periodic();
    }
}
