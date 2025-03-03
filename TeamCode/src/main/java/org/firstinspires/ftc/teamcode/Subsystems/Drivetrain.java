package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain extends SubsystemBase {
    private final Motor frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;
    private final Telemetry telemetry;
    public final MecanumDrive mecanumDrive;
    private final GamepadEx gamepadEx;
    public Pose2d robotPose;
    public MecanumDriveOdometry odometry = new MecanumDriveOdometry(
            new MecanumDriveKinematics(new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()), new Rotation2d());
    private MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds();

    public Drivetrain(Telemetry telemetry, Motor[] motors, IMU imu, GamepadEx gamepadEx) {
        frontLeft = motors[0];
        frontRight = motors[1];
        backLeft = motors[2];
        backRight = motors[3];
        this.imu = imu;
        this.telemetry = telemetry;
        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.gamepadEx = gamepadEx;

        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        mecanumDrive.setRightSideInverted(false);
    }

    public void drive(boolean fieldOriented) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if(fieldOriented) {
            mecanumDrive.driveFieldCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX(), heading);
        } else {
            mecanumDrive.driveRobotCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX());
        }
    }

    public GamepadEx getGamepadEx() {
        return gamepadEx;
    }

    @Override
    public void periodic() {
        robotPose = odometry.updateWithTime((double) System.currentTimeMillis() / 1000, Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), wheelSpeeds);

        telemetry.addLine("MOTOR POWERS");
        telemetry.addData("FrontLeft Power", frontLeft.get());
        telemetry.addData("FrontRight Power Level", frontRight.get());
        telemetry.addData("BackLeft Power", backLeft.get());
        telemetry.addData("BackRight Power", backRight.get());

        telemetry.addLine("IMU");
        telemetry.addData("IMU orientation", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.update();
    }
}
