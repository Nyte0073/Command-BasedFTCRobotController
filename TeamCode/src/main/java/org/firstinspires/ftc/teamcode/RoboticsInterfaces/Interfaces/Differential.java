package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public abstract class Differential extends SubsystemBase implements Driveable {

    /**Drives the robot either in a tank or arcade style depending the value of the
     * {@code driveType} field inputted into the {@code DifferentialVector} field
     * "differentialVector".*/
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

    /**Returns the {@code DifferentialDrive} object sent over by any classes that extend this class
     * to make a differential drivetrain.*/
    public DifferentialDrive getDifferentialDrive(DifferentialVector differentialVector) {
        return new DifferentialDrive(differentialVector.leftGroup, differentialVector.rightGroup);
    }

    /**Stops the robot's motors.*/
    @Override
    public void stop() {
        stopMotors();
    }

    /**Updates the telemetry all at once using a {@code Map<String, Object>} and looping and adding all the data from it to the telemetry
     * by looping through all its contents.*/
    @Override
    public void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap) {
        for(String s : telemetryMap.keySet()) {
            telemetry.addData(s, telemetryMap.get(s));
        }
    }

    /**Resets the yaw angle of the robot so that the robot now thinks current heading is 0 degrees,
     * and not whatever it was before.*/
    @Override
    public void resetGyro(IMU imu) {
        imu.resetYaw();
    }

    /**Returns the {@code DifferentialVector} object sent over by other classes that extend this class to create
     * a differential drivetrain.*/
    public abstract DifferentialVector getDifferentialVector();

    /**Sets the powers of all the motors to 0, stopping them completely.*/
    public abstract void stopMotors();
}
