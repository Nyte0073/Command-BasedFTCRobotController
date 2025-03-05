package org.firstinspires.ftc.teamcode.MainTeamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import java.util.List;

@Autonomous(name = "Auto", group = "teamcode")
public class Auto extends CommandOpMode {
    Drivetrain drivetrain;

    Motor[] motors = new Motor[] {
            new Motor(hardwareMap, Constants.MotorConstants.frontLeftMotor),
            new Motor(hardwareMap, Constants.MotorConstants.frontRightMotor),
            new Motor(hardwareMap, Constants.MotorConstants.backLeftMotor),
            new Motor(hardwareMap, Constants.MotorConstants.backRightMotor)
    };

    GamepadEx gamepadEx;

    IMU imu;

    MecanumControllerCommand command;

    @Override
    public void initialize() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        gamepadEx = new GamepadEx(gamepad1);
        drivetrain = new Drivetrain(telemetry, motors, imu, gamepadEx);

        Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                new TrajectoryConfig(2.0, 2.0)
        );

        Consumer <MecanumDriveWheelSpeeds> outputSpeeds =
                (m) -> {
                    telemetry.addData("Front Left Wheel Speeds", m.frontLeftMetersPerSecond);
                    telemetry.addData("Front Right Wheel Speeds", m.frontRightMetersPerSecond);
                    telemetry.addData("Back Left Wheel Speeds", m.rearLeftMetersPerSecond);
                    telemetry.addData("Back Right Wheel Speeds", m.rearRightMetersPerSecond);
                };

        command = new MecanumControllerCommand(
                firstTrajectory,
                drivetrain::getRobotPose,
                drivetrain.getKinematics(),
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints()),
                2.0,
                outputSpeeds
                );

        command.schedule();
    }
}
