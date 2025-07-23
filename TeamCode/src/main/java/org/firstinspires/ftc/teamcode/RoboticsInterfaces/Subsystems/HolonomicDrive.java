package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Holonomic;

import java.util.HashMap;
import java.util.List;

/**Class to call when wanting to run a holonomic-type drivetrain, that either runs in with an X-pattern or H-pattern drivetrain.*/
public class HolonomicDrive extends Holonomic {

    /**Reference to the type of drivetrain you want to use with this class.*/
    String driveType;

    /**Reference the driving motors for either the H-pattern or X-pattern drivetrain.*/
    Motor[] HDriveMotors, XDriveMotors;

    /**Reference to the human driver's Xbox controller.*/
    GamepadEx gamepadEx;

    /**The angles that the drivetrain's motors are set at on the robot.*/
    double[] motorAngles;

    /**The robot's IMU system. Returns the current orientation of the robot in degrees.*/
    IMU imu;

    /**Whether the robot is going to drive field-oriented or not.*/
    boolean fieldOriented;

    /**Reference to the robot's telemetry system for sending information from the robot to the control hub.*/
    Telemetry telemetry;

    /**Constructs a new {@code HolonomicDrive()} with an initialized {@code driveType}, {@code HDriveMotors} array, {@code XDriveMotors} array,
     * {@code GamepadEx}, {@code motorAngles} array, {@code IMU} and {@code fieldOriented} state.*/
    public HolonomicDrive(String driveType, Motor[] HDriveMotors, Motor[] XDriveMotors, GamepadEx gamepadEx, double[] motorAngles, IMU imu, boolean fieldOriented) {
        this.driveType = driveType;
        this.HDriveMotors = HDriveMotors;
        this.XDriveMotors = XDriveMotors;
        this.gamepadEx = gamepadEx;
        this.motorAngles = motorAngles;
        this.imu = imu;
        this.fieldOriented = fieldOriented;
    }

    /**Returns the {@code driveType} of the robot.*/
    @Override
    public String getDriveType() {
        return driveType;
    }

    /**Returns the {@code Motor} arrays for the X-pattern and H-pattern drivetrains.*/
    @Override
    public List<Motor[]> getHolonomicMotors() {
        return List.of(HDriveMotors, XDriveMotors);
    }

    /**Returns the {@code motorAngles} array for getting the angular positions of all the motors relative to the robot.*/
    @Override
    public double[] getHolonomicMotorAngles() {
        return motorAngles;
    }

    /**Returns whether the robot is field oriented or not in terms of driving.*/
    @Override
    public boolean getFieldOriented() {
        return fieldOriented;
    }

    /**Returns the {@code GamepadEx} reference that the robot is currently using.*/
    @Override
    public GamepadEx getGamepadEx() {
        return gamepadEx;
    }

    /**Returns the {@code imu} variable that the robot is currently using to calculate its orientation.*/
    @Override
    public IMU getIMU() {
        return imu;
    }

    /**Stops the robot's motors by setting their motor power to 0.*/
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

    /**Updates the telemetry with all the current information about the robot.*/
    @Override
    public void periodic() {
        updateTelemetry(
               telemetry, new HashMap<String, Object>() {{
                   put("Motor Angles", motorAngles);
                }}
        );
    }
}