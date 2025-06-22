package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public abstract class Differential extends SubsystemBase implements Driveable {
    @Override
    public void drive() {
        DifferentialVector differentialVector = getDifferentialVector();
        DifferentialDrive differentialDrive = getDifferentialDrive(differentialVector);
        GamepadEx gamepadEx = differentialVector.gamepadEx;

        double leftSpeed = -gamepadEx.getLeftY();
        double rightSpeed = -gamepadEx.getRightY();
        double turnSpeed = gamepadEx.getRightX();

        switch(differentialVector.driveType) {
            case "tankDrive":
                differentialDrive.tankDrive(leftSpeed, rightSpeed);
                break;

            case "arcade":
                differentialDrive.arcadeDrive(leftSpeed, turnSpeed);
                break;
        }
    }

    public DifferentialDrive getDifferentialDrive(DifferentialVector differentialVector) {
        return new DifferentialDrive(differentialVector.leftGroup, differentialVector.rightGroup);
    }

    @Override
    public void stop() {
        stopMotors();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap) {
        for(String s : telemetryMap.keySet()) {
            telemetry.addData(s, telemetryMap.get(s));
        }
    }

    @Override
    public void resetGyro(IMU imu) {
        imu.resetYaw();
    }

    public abstract DifferentialVector getDifferentialVector();

    public abstract void stopMotors();
}
