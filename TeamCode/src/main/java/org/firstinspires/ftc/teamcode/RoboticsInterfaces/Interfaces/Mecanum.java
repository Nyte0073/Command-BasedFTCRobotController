package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Map;

public abstract class Mecanum extends SubsystemBase implements Driveable {

    /**Drives the robot either in a field-oriented or non field-oriented way.*/
    @Override
    public void drive() {
        MecanumDrive mecanumDrive = getMecanumDrive();
        if(getFieldOriented()) {
            mecanumDrive.driveFieldCentric(getGamepadEx().getLeftX(), -getGamepadEx().getLeftY(),
                    getGamepadEx().getRightX(), getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        } else {
            mecanumDrive.driveRobotCentric(getGamepadEx().getLeftX(), -getGamepadEx().getLeftY(),
                    getGamepadEx().getRightX());
        }
    }

    /**Stops the robot's turning and driving motors by setting their motor powers to 0.*/
    @Override
    public void stop() {
        stopMotors();
    }

    /**Resets the yaw angle of the robot's IMU orientation system.*/
    @Override
    public void resetGyro(IMU imu) {
        imu.resetYaw();
    }

    /**Updates the telemetry al at once by calling {@code telemetry.addData()} to everything in a Map that stores all the data to be
     * written to the {@code telemetry} variable.*/
    @Override
    public void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap) {
        for(String s : telemetryMap.keySet()) {
            telemetry.addData(s, telemetryMap.get(s));
        }
    }

    /**Sets the motor powers of the driving/turning motors to 0.*/
    public abstract void stopMotors();

    /**Returns the reference to the robot's four mecanum wheels and their access*/
    public abstract MecanumDrive getMecanumDrive();

    /**Returns whether the robot will drive field-oriented or robot-oriented.*/
    public abstract boolean getFieldOriented();

    /**Returns the {@code gamepadEx} variable from extending classes, to use in the {@code drive()} method.*/
    public abstract GamepadEx getGamepadEx();

    /**Returns the {@code imu} variable from extending classes to use in the {@code drive()} method.*/
    public abstract IMU getIMU();
}
