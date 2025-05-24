package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Map;

public abstract class Mecanum extends SubsystemBase implements Driveable {
    @Override
    public void drive() {
        if(getFieldOriented()) {
            getMecanumDrive().driveFieldCentric(getGamepadEx().getLeftX(), -getGamepadEx().getLeftY(),
                    getGamepadEx().getRightX(), getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        } else {
            getMecanumDrive().driveRobotCentric(getGamepadEx().getLeftX(), -getGamepadEx().getLeftY(),
                    getGamepadEx().getRightX());
        }
    }

    @Override
    public void stop() {
        stopMotors();
    }

    @Override
    public void resetGyro(IMU imu) {
        imu.resetYaw();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap) {
        for(String s : telemetryMap.keySet()) {
            telemetry.addData(s, telemetryMap.get(s));
        }
    }

    public abstract void stopMotors();

    public abstract MecanumDrive getMecanumDrive();

    public abstract boolean getFieldOriented();

    public abstract GamepadEx getGamepadEx();

    public abstract IMU getIMU();
}
