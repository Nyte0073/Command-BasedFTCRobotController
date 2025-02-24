package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain extends SubsystemBase {
    private final Motor frontLeft, frontRight, backLeft, backRight;
    final IMU imu;
    Telemetry telemetry;
    private final MecanumDrive mecanumDrive;
    GamepadEx gamepadEx;

    public Drivetrain(Telemetry telemetry, Motor[] motors, IMU imu, GamepadEx gamepadEx) {
        frontLeft = motors[0];
        frontRight = motors[1];
        backLeft = motors[2];
        backRight = motors[3];
        this.imu = imu;
        this.telemetry = telemetry;
        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.gamepadEx = gamepadEx;
    }

    public void drive(boolean fieldOriented) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw();
        if(fieldOriented) {
            mecanumDrive.driveFieldCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX(), heading);
        } else {
            mecanumDrive.driveRobotCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX());
        }
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }

    @Override
    public void periodic() {
        telemetry.addLine("MOTOR POWERS");
        telemetry.addData("FrontLeft Power", frontLeft.get());
        telemetry.addData("FrontRight Power Level", frontRight.get());
        telemetry.addData("BackLeft Power", backLeft.get());
        telemetry.addData("BackRight Power", backRight.get());
        telemetry.update();
    }
}
