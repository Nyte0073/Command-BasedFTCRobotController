package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Mecanum;
import java.util.HashMap;

public class MecanumDrive extends Mecanum {

    /***/
    com.arcrobotics.ftclib.drivebase.MecanumDrive mecanumDrive;

    Motor frontLeft, frontRight, backLeft, backRight;

    Telemetry telemetry;

    IMU imu;

    boolean fieldOriented;

    GamepadEx gamepadEx;

    public MecanumDrive(Motor[] motors, Telemetry telemetry, IMU imu, boolean fieldOriented, GamepadEx gamepadEx) {
        frontLeft = motors[0];
        frontRight = motors[1];
        backLeft = motors[2];
        backRight = motors[3];
        mecanumDrive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.telemetry = telemetry;
        this.imu = imu;
        this.fieldOriented = fieldOriented;
        this.gamepadEx = gamepadEx;
    }

    @Override
    public void stopMotors() {
        mecanumDrive.stop();
    }

    @Override
    public com.arcrobotics.ftclib.drivebase.MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    @Override
    public boolean getFieldOriented() {
        return fieldOriented;
    }

    @Override
    public GamepadEx getGamepadEx() {
        return gamepadEx;
    }

    @Override
    public IMU getIMU() {
        return imu;
    }

    @Override
    public void periodic() {
        updateTelemetry(telemetry, new HashMap<String, Object>() {{
            put("IMU Orientation", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }});
    }
}
