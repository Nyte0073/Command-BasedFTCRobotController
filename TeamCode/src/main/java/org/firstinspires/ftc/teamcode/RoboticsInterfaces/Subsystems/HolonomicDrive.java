package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Holonomic;
import java.util.List;

public class HolonomicDrive extends Holonomic {

    String driveType;

    Motor[] HDriveMotors, XDriveMotors;

    GamepadEx gamepadEx;

    double[] motorAngles;

    IMU imu;

    boolean fieldOriented;

    public HolonomicDrive(String driveType, Motor[] HDriveMotors, Motor[] XDriveMotors, GamepadEx gamepadEx, double[] motorAngles, IMU imu, boolean fieldOriented) {
        this.driveType = driveType;
        this.HDriveMotors = HDriveMotors;
        this.XDriveMotors = XDriveMotors;
        this.gamepadEx = gamepadEx;
        this.motorAngles = motorAngles;
        this.imu = imu;
        this.fieldOriented = fieldOriented;
    }

    @Override
    public String getDriveType() {
        return driveType;
    }

    @Override
    public List<Motor[]> getHolonomicMotors() {
        return List.of(HDriveMotors, XDriveMotors);
    }

    @Override
    public double[] getHolonomicMotorAngles() {
        return motorAngles;
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
    public void stopMotors() {
        switch(driveType) {
            case "HDrive":
                for(Motor m : HDriveMotors) {
                    m.set(0);
                }
                break;

            case "XDrive":
                for(Motor m : XDriveMotors) {
                    m.set(0);
                }
                break;
        }
    }
}